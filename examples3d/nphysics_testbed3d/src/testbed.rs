use std::env;
use std::path::Path;
use time;
use glfw::{self, MouseButton, Key, Action, WindowEvent};
use num::Bounded;

use na::{self, Point2, Point3, Vector3, Translation3, Isometry3};
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::text::Font;
use kiss3d::loader::obj;
use ncollide::shape::{Cuboid, Ball};
use ncollide::query::{self, Ray};
use ncollide::world::CollisionGroups;
use nphysics3d::detection::constraint::Constraint;
use nphysics3d::detection::joint::{Anchor, Fixed, Joint};
use nphysics3d::object::{RigidBody, RigidBodyHandle, SensorHandle, WorldObject};
use nphysics3d::world::World;
use nphysics3d::Rc;
use engine::{GraphicsManager, GraphicsManagerHandle};


fn usage(exe_name: &str) {
    println!("Usage: {} [OPTION] ", exe_name);
    println!("");
    println!("Options:");
    println!("    --help  - prints this help message and exits.");
    println!("    --pause - do not start the simulation right away.");
    println!("");
    println!("The following keyboard commands are supported:");
    println!("    t      - pause/continue the simulation.");
    println!("    s      - pause then execute only one simulation step.");
    println!("    1      - launch a ball.");
    println!("    2      - launch a cube.");
    println!("    3      - launch a fast cube using continuous collision detection.");
    println!("    TAB    - switch camera mode (first-person or arc-ball).");
    println!("    SHIFT + right click - launch a fast cube using continuous collision detection.");
    println!("    CTRL + left click + drag - select and drag an object using a ball-in-socket joint.");
    println!("    SHIFT + left click - remove an object.");
    println!("    arrows - move around when in first-person camera mode.");
    println!("    space  - switch wireframe mode. When ON, the contacts points and normals are displayed.");
    println!("    b      - draw the bounding boxes.");
}

pub struct Testbed {
    world:    World<f32>,
    window:   Window,
    graphics: GraphicsManagerHandle,
}

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let window   = Window::new("nphysics: 3d demo");

        Testbed {
            world:    World::new(),
            window:   window,
            graphics: Rc::new(graphics)
        }
    }

    pub fn new(world: World<f32>) -> Testbed {
        let mut res = Testbed::new_empty();

        res.set_world(world);

        res
    }

    pub fn set_world(&mut self, world: World<f32>) {
        self.world = world;

        self.graphics.borrow_mut().clear(&mut self.window);

        for rb in self.world.rigid_bodies() {
            self.graphics.borrow_mut().add(&mut self.window, WorldObject::RigidBody(rb.clone()));
        }

        for sensor in self.world.sensors() {
            self.graphics.borrow_mut().add(&mut self.window, WorldObject::Sensor(sensor.clone()));
        }
    }

    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.graphics.borrow_mut().look_at(eye, at);
    }

    pub fn set_rigid_body_color(&mut self, rb: &RigidBodyHandle<f32>, color: Point3<f32>) {
        self.graphics.borrow_mut().set_rigid_body_color(rb, color);
    }

    pub fn set_sensor_color(&mut self, sensor: &SensorHandle<f32>, color: Point3<f32>) {
        self.graphics.borrow_mut().set_sensor_color(sensor, color);
    }

    pub fn world(&self) -> &World<f32> {
        &self.world
    }

    pub fn graphics(&self) -> GraphicsManagerHandle {
        self.graphics.clone()
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path    = Path::new(path);
        let empty   = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").ok().expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().unwrap().to_owned().unwrap();
            let indices  = m.faces().read().unwrap().to_owned().unwrap();

            let mut flat_indices = Vec::new();

            for i in indices.into_iter() {
                flat_indices.push(i.x as usize);
                flat_indices.push(i.y as usize);
                flat_indices.push(i.z as usize);
            }

            let m = (vertices, flat_indices);

            res.push(m);
        }

        res
    }

    pub fn run(&mut self) {
        let mut args    = env::args();
        let mut running = RunMode::Running;

        if args.len() > 1 {
            let exname = args.next().unwrap();
            for arg in args {
                if &arg[..] == "--help" || &arg[..] == "-h" {
                    usage(&exname[..]);
                    return;
                }
                else if &arg[..] == "--pause" {
                    running = RunMode::Stop;
                }
            }
        }

        let font_mem       = include_bytes!("Inconsolata.otf");
        let font           = Font::from_memory(font_mem, 60);
        let mut draw_colls = false;

        let mut cursor_pos = Point2::new(0.0f32, 0.0);
        let mut grabbed_object: Option<RigidBodyHandle<f32>> = None;
        let mut grabbed_object_joint: Option<Rc<Fixed<f32>>> = None;
        let mut grabbed_object_plane: (Point3<f32>, Vector3<f32>) = (Point3::origin(), na::zero());


        self.window.set_framerate_limit(Some(60));
        self.window.set_light(Light::StickToCamera);

        while !self.window.should_close() {
            for mut event in self.window.events().iter() {
                match event.value {
                    WindowEvent::MouseButton(MouseButton::Button2, Action::Press, glfw::Control) => {
                        let mut graphics = self.graphics.borrow_mut();
                        let geom   = Cuboid::new(Vector3::new(0.5f32, 0.5f32, 0.5f32));
                        let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                        let size = self.window.size();
                        let (pos, dir) = graphics.camera().unproject(&cursor_pos, &size);

                        rb.set_translation(Translation3::from_vector(pos.coords));
                        rb.set_lin_vel(dir * 1000.0f32);

                        let body = self.world.add_rigid_body(rb);
                        self.world.add_ccd_to(&body, 1.0, false);
                        graphics.add(&mut self.window, WorldObject::RigidBody(body));
                    },
                    WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                        if modifier.contains(glfw::Shift) {
                            // XXX: huge and uggly code duplication
                            let size = self.window.size();
                            let (pos, dir) = self.graphics.borrow().camera().unproject(&cursor_pos, &size);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            let all_groups = &CollisionGroups::new();
                            // FIXME: add the Sensor group to the blacklist so that they are not
                            // reported for ray-casting.
                            for (b, inter) in self.world
                                                  .collision_world()
                                                  .interferences_with_ray(&ray, all_groups) {
                                if  inter.toi < mintoi {
                                    if let &WorldObject::RigidBody(ref rb) = &b.data {
                                        mintoi = inter.toi;
                                        minb   = Some(rb.clone());
                                    }
                                }
                            }

                            if minb.is_some() {
                                let b = minb.as_ref().unwrap();
                                if b.borrow().can_move() {
                                    self.world.remove_rigid_body(b);
                                    self.graphics.borrow_mut().remove(&mut self.window, &WorldObject::RigidBody(b.clone()));
                                }
                            }

                            event.inhibited = true;
                        }
                        else if modifier.contains(glfw::Control) {
                            match grabbed_object {
                                Some(ref rb) => {
                                    for n in self.graphics.borrow_mut().rigid_body_nodes_mut(rb).unwrap().iter_mut() {
                                        n.unselect()
                                    }
                                },
                                None => { }
                            }

                            // XXX: huge and uggly code duplication
                            let size = self.window.size();
                            let (pos, dir) = self.graphics.borrow().camera().unproject(&cursor_pos, &size);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            let all_groups = CollisionGroups::new();
                            for (b, inter) in self.world
                                                  .collision_world()
                                                  .interferences_with_ray(&ray, &all_groups) {
                                if  inter.toi < mintoi {
                                    if let &WorldObject::RigidBody(ref rb) = &b.data {
                                        mintoi = inter.toi;
                                        minb   = Some(rb.clone());
                                    }
                                }
                            }

                            if minb.is_some() {
                                let b = minb.as_ref().unwrap();
                                if b.borrow().can_move() {
                                    grabbed_object = Some(b.clone())
                                }
                            }

                            match grabbed_object {
                                Some(ref b) => {
                                    for n in self.graphics.borrow_mut().rigid_body_nodes_mut(b).unwrap().iter_mut() {
                                        match grabbed_object_joint {
                                            Some(ref j) => self.world.remove_fixed(j),
                                            None        => { }
                                        }

                                        let attach2_pos = ray.origin + ray.dir * mintoi;
                                        let attach2 = Isometry3::new(attach2_pos.coords, na::zero());
                                        let attach1 = b.borrow().position().inverse() * attach2;

                                        let anchor1 = Anchor::new(Some(minb.as_ref().unwrap().clone()), attach1);
                                        let anchor2 = Anchor::new(None, attach2);
                                        let joint   = Fixed::new(anchor1, anchor2);
                                        grabbed_object_plane = (attach2_pos, -ray.dir);
                                        grabbed_object_joint = Some(self.world.add_fixed(joint));
                                        // Add a joint.
                                        n.select()
                                    }
                                },
                                None => { }
                            }

                            event.inhibited = true;
                        }
                    },
                    WindowEvent::MouseButton(_, Action::Release, _) => {
                        let mut graphics = self.graphics.borrow_mut();
                        match grabbed_object {
                            Some(ref b) => {
                                for n in graphics.rigid_body_nodes_mut(b).unwrap().iter_mut() {
                                    n.unselect()
                                }
                            },
                            None => { }
                        }

                        match grabbed_object_joint {
                            Some(ref j) => self.world.remove_fixed(j),
                            None    => { }
                        }

                        grabbed_object       = None;
                        grabbed_object_joint = None;
                    },
                    WindowEvent::CursorPos(x, y) => {
                        cursor_pos.x = x as f32;
                        cursor_pos.y = y as f32;

                        // update the joint
                        match grabbed_object_joint {
                            Some(ref j) => {
                                let size = self.window.size();
                                let (pos, dir) = self.graphics.borrow().camera().unproject(&cursor_pos, &size);
                                let (ref ppos, ref pdir) = grabbed_object_plane;

                                match query::ray_internal::plane_toi_with_ray(ppos, pdir, &Ray::new(pos, dir)) {
                                    Some(inter) => {
                                        j.borrow_mut().set_local2(Isometry3::new((pos + dir * inter).coords, na::zero()))
                                    },
                                    None => { }
                                }

                            },
                            None => { }
                        }

                        event.inhibited =
                            self.window.glfw_window().get_key(Key::RightShift)   != Action::Release ||
                            self.window.glfw_window().get_key(Key::LeftShift)    != Action::Release ||
                            self.window.glfw_window().get_key(Key::RightControl) != Action::Release ||
                            self.window.glfw_window().get_key(Key::LeftControl)  != Action::Release;
                    },
                    WindowEvent::Key(Key::Tab, _, Action::Release, _) => self.graphics.borrow_mut().switch_cameras(),
                    WindowEvent::Key(Key::T, _,   Action::Release, _) => {
                        if running == RunMode::Stop {
                            running = RunMode::Running;
                        }
                        else {
                            running = RunMode::Stop;
                        }
                    },
                    WindowEvent::Key(Key::S, _, Action::Release, _) => running = RunMode::Step,
                    WindowEvent::Key(Key::B, _, Action::Release, _) => {
                        // XXX: there is a bug on kiss3d with the removal of objects.
                        // draw_aabbs = !draw_aabbs;
                        // if draw_aabbs {
                        //     graphics.enable_aabb_draw(&mut self.window);
                        // }
                        // else {
                        //     graphics.disable_aabb_draw(&mut self.window);
                        // }
                    },
                    WindowEvent::Key(Key::Space, _, Action::Release, _) => {
                        let mut graphics = self.graphics.borrow_mut();
                        draw_colls = !draw_colls;
                        for rb in self.world.rigid_bodies() {
                            // FIXME: ugly clone.
                            if let Some(ns) = graphics.rigid_body_nodes_mut(rb) {
                                for n in ns.iter_mut() {
                                    if draw_colls {
                                        n.scene_node_mut().set_lines_width(1.0);
                                        n.scene_node_mut().set_surface_rendering_activation(false);
                                    }
                                    else {
                                        n.scene_node_mut().set_lines_width(0.0);
                                        n.scene_node_mut().set_surface_rendering_activation(true);
                                    }
                                }
                            }
                        }
                    },
                    WindowEvent::Key(Key::Num1, _, Action::Press, _) => {
                        let mut graphics = self.graphics.borrow_mut();
                        let geom   = Ball::new(0.5f32);
                        let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                        let cam_transfom;

                        {
                            let cam      = graphics.camera();
                            cam_transfom = cam.view_transform().inverse()
                        }

                        rb.append_translation(&cam_transfom.translation);

                        let front = -cam_transfom.rotation * Vector3::z();

                        rb.set_lin_vel(front * 40.0f32);

                        let body = self.world.add_rigid_body(rb);
                        graphics.add(&mut self.window, WorldObject::RigidBody(body));
                    },
                    WindowEvent::Key(Key::Num2, _, Action::Press, _) => {
                        let mut graphics = self.graphics.borrow_mut();
                        let geom   = Cuboid::new(Vector3::new(0.5f32, 0.5, 0.5));
                        let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                        let cam_transform;

                        {
                            let cam = graphics.camera();
                            cam_transform = cam.view_transform().inverse()
                        }

                        rb.append_translation(&cam_transform.translation);

                        let front = -cam_transform.rotation * Vector3::z();

                        rb.set_lin_vel(front * 40.0f32);

                        let body = self.world.add_rigid_body(rb);
                        graphics.add(&mut self.window, WorldObject::RigidBody(body));
                    }
                    _ => { }
                }
            }

            let dt;

            if running != RunMode::Stop {
                let before = time::precise_time_s();
                self.world.step(0.016);
                dt = time::precise_time_s() - before;

                self.graphics.borrow_mut().draw();
            }
            else {
                dt = 0.0;
            }

            if running == RunMode::Step {
                running = RunMode::Stop;
            }

            if draw_colls {
                self.graphics.borrow_mut().draw_positions(&mut self.window);
                draw_collisions(&mut self.window, &mut self.world);
            }

            let color = Point3::new(1.0, 1.0, 1.0);

            if running != RunMode::Stop {
                self.window.draw_text(&format!("Time: {:.*}sec.", 4, dt)[..], &Point2::origin(), &font, &color);
            }
            else {
                self.window.draw_text("Paused", &Point2::origin(), &font, &color);
            }

            self.window.render_with_camera(self.graphics.borrow_mut().camera_mut());
        }
    }
}

#[derive(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step
}

fn draw_collisions(window: &mut Window, physics: &mut World<f32>) {
    let mut collisions = Vec::new();

    physics.constraints(&mut collisions);

    for c in collisions.iter() {
        match *c {
            Constraint::RBRB(_, _, ref c) => {
                window.draw_line(&c.world1, &c.world2, &Point3::new(1.0, 0.0, 0.0));

                let center = na::center(&c.world1, &c.world2);
                let end    = center + c.normal * 0.4f32;
                window.draw_line(&center, &end, &Point3::new(0.0, 1.0, 1.0))
            },
            Constraint::BallInSocket(ref bis) => {
                let bbis = bis.borrow();
                window.draw_line(&bbis.anchor1_pos(), &bbis.anchor2_pos(), &Point3::new(0.0, 1.0, 0.0));
            },
            Constraint::Fixed(ref f) => {
                // FIXME: draw the rotation too
                let p1 = Point3::from_coordinates(f.borrow().anchor1_pos().translation.vector);
                let p2 = Point3::from_coordinates(f.borrow().anchor2_pos().translation.vector);

                window.draw_line(&p1, &p2, &Point3::new(0.0, 1.0, 0.0));
            }
        }
    }
}
