use std::env;
use std::rc::Rc;
use std::cell::RefCell;
use std::path::Path;
use std::collections::HashMap;
use time;
use glfw::{Action, Key, Modifiers, MouseButton, WindowEvent};
use num::Bounded;

use na::{self, Isometry3, Point2, Point3, Vector3};
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::text::Font;
use kiss3d::loader::obj;
use ncollide3d::query::{self, Ray};
use ncollide3d::world::CollisionGroups;
use ncollide3d::utils::GenerationalId;
use nphysics3d::object::BodyHandle;
use nphysics3d::world::World;
use nphysics3d::joint::{ConstraintHandle, FixedConstraint};
use engine::{GraphicsManager, GraphicsManagerHandle};

#[derive(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step,
}

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
    println!(
        "    CTRL + left click + drag - select and drag an object using a ball-in-socket joint."
    );
    println!("    SHIFT + left click - remove an object.");
    println!("    arrows - move around when in first-person camera mode.");
    println!("    space  - switch wireframe mode. When ON, the contacts points and normals are displayed.");
    println!("    b      - draw the bounding boxes.");
}

pub struct Testbed {
    world: World<f32>,
    window: Window,
    graphics: GraphicsManagerHandle,
    nsteps: usize,
    callbacks: Vec<Box<Fn(&mut World<f32>, f32)>>,
    time: f32,
    physics_timer: f64,
    hide_counters: bool,
    persistant_contacts: HashMap<GenerationalId, bool>,
}

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let mut window = Window::new("nphysics: 3d demo");
        let world = World::new();

        window.set_background_color(0.9, 0.9, 0.9);

        Testbed {
            world: world,
            callbacks: Vec::new(),
            window: window,
            graphics: Rc::new(RefCell::new(graphics)),
            nsteps: 1,
            time: 0.0,
            physics_timer: 0.0,
            hide_counters: false,
            persistant_contacts: HashMap::new(),
        }
    }

    pub fn new(world: World<f32>) -> Testbed {
        let mut res = Testbed::new_empty();

        res.set_world(world);

        res
    }

    pub fn set_number_of_steps_per_frame(&mut self, nsteps: usize) {
        self.nsteps = nsteps
    }

    pub fn hide_performance_counters(&mut self) {
        self.hide_counters = true;
    }

    pub fn show_performance_counters(&mut self) {
        self.hide_counters = false;
    }

    pub fn set_world(&mut self, world: World<f32>) {
        self.world = world;
        self.world.enable_performance_counters();

        self.graphics.borrow_mut().clear(&mut self.window);

        for co in self.world.colliders() {
            self.graphics
                .borrow_mut()
                .add(&mut self.window, co.handle(), &self.world);
        }

        // for sensor in self.world.sensors() {
        //     self.graphics.borrow_mut().add(&mut self.window, WorldObject::Sensor(sensor.clone()));
        // }
    }

    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.graphics.borrow_mut().look_at(eye, at);
    }

    pub fn set_body_color(&mut self, world: &World<f32>, body: BodyHandle, color: Point3<f32>) {
        self.graphics
            .borrow_mut()
            .set_body_color(world, body, color);
    }

    // pub fn set_sensor_color(&mut self, sensor: &SensorHandle<f32>, color: Point3<f32>) {
    //     self.graphics.borrow_mut().set_sensor_color(sensor, color);
    // }

    pub fn world(&self) -> &World<f32> {
        &self.world
    }

    pub fn graphics(&self) -> GraphicsManagerHandle {
        self.graphics.clone()
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path = Path::new(path);
        let empty = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "")
            .ok()
            .expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().unwrap().to_owned().unwrap();
            let indices = m.faces().read().unwrap().to_owned().unwrap();

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

    pub fn add_callback<F: Fn(&mut World<f32>, f32) + 'static>(&mut self, callback: F) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn run(&mut self) {
        let mut args = env::args();
        let mut running = RunMode::Running;

        if args.len() > 1 {
            let exname = args.next().unwrap();
            for arg in args {
                if &arg[..] == "--help" || &arg[..] == "-h" {
                    usage(&exname[..]);
                    return;
                } else if &arg[..] == "--pause" {
                    running = RunMode::Stop;
                }
            }
        }

        let font_mem = include_bytes!("Inconsolata.otf");
        let font = Font::from_memory(font_mem, 60);
        let mut draw_colls = false;

        let mut cursor_pos = Point2::new(0.0f32, 0.0);
        let mut grabbed_object: Option<BodyHandle> = None;
        let mut grabbed_object_joint: Option<ConstraintHandle> = None;
        let mut grabbed_object_plane: (Point3<f32>, Vector3<f32>) = (Point3::origin(), na::zero());

        self.window.set_framerate_limit(Some(60));
        self.window.set_light(Light::StickToCamera);

        while !self.window.should_close() {
            for mut event in self.window.events().iter() {
                match event.value {
                    //         WindowEvent::MouseButton(MouseButton::Button2, Action::Press, Key::LeftControl) |
            //         WindowEvent::MouseButton(MouseButton::Button2, Action::Press, Key::RightControl)  => {
            //             let mut graphics = self.graphics.borrow_mut();
            //             let geom   = Cuboid::new(Vector3::new(0.5f32, 0.5f32, 0.5f32));
            //             let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

            //             let size = self.window.size();
            //             let (pos, dir) = graphics.camera().unproject(&cursor_pos, &size);

            //             rb.set_translation(Translation3::from_vector(pos.coords));
            //             rb.set_lin_vel(dir * 1000.0f32);

            //             let body = self.world.add_rigid_body(rb);
            //             self.world.add_ccd_to(&body, 1.0, false);
            //             graphics.add(&mut self.window, WorldObject::RigidBody(body));
            //         },
                    WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                        if modifier.contains(Modifiers::Shift) {
                            // XXX: huge and uggly code duplication for the ray cast.
                            let size = self.window.size();
                            let (pos, dir) = self.graphics
                                .borrow()
                                .camera()
                                .unproject(&cursor_pos, &size);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut mintoi = Bounded::max_value();
                            let mut minb = None;

                            let all_groups = CollisionGroups::new();
                            for (b, inter) in self.world
                                .collision_world()
                                .interferences_with_ray(&ray, &all_groups)
                            {
                                if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                    mintoi = inter.toi;
                                    minb = Some(b.data().body());
                                }
                            }

                            if let Some(body) = minb {
                                if !body.is_ground() {
                                    let mut bgraphics = self.graphics.borrow_mut();
                                    if !modifier.contains(Modifiers::Control) {
                                        bgraphics.remove_body_nodes(
                                            &self.world,
                                            &mut self.window,
                                            body,
                                        );
                                        self.world.remove_bodies(&[body]);
                                    } else {
                                        if self.world.multibody_link(body).is_some() {
                                            let key = bgraphics.remove_body_part_nodes(
                                                &self.world,
                                                &mut self.window,
                                                body,
                                            );
                                            self.world.remove_multibody_links(&[body]);
                                            // FIXME: this is a bit ugly.
                                            bgraphics
                                                .update_after_body_key_change(&self.world, key);
                                        }
                                    }
                                }
                            }

                            event.inhibited = true;
                        } else if modifier.contains(Modifiers::Control) {
                            match grabbed_object {
                                Some(body) => for n in self.graphics
                                    .borrow_mut()
                                    .body_nodes_mut(&self.world, body)
                                    .unwrap()
                                    .iter_mut()
                                {
                                    n.unselect()
                                },
                                None => {}
                            }

                            // XXX: huge and uggly code duplication for the ray cast.
                            let size = self.window.size();
                            let (pos, dir) = self.graphics
                                .borrow()
                                .camera()
                                .unproject(&cursor_pos, &size);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut mintoi = Bounded::max_value();
                            let mut minb = None;

                            let all_groups = CollisionGroups::new();
                            for (b, inter) in self.world
                                .collision_world()
                                .interferences_with_ray(&ray, &all_groups)
                            {
                                if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                    mintoi = inter.toi;
                                    minb = Some(b.data().body());
                                }
                            }

                            if let Some(body) = minb {
                                if self.world.body(body).status_dependent_ndofs() != 0 {
                                    grabbed_object = minb;
                                    for n in self.graphics
                                        .borrow_mut()
                                        .body_nodes_mut(&self.world, body)
                                        .unwrap()
                                        .iter_mut()
                                    {
                                        if let Some(joint) = grabbed_object_joint {
                                            self.world.remove_constraint(joint);
                                        }

                                        let body_pos = self.world.body_part(body).position();
                                        let attach1_pos = ray.origin + ray.dir * mintoi;
                                        let attach1 =
                                            Isometry3::new(attach1_pos.coords, na::zero());
                                        let attach2 = body_pos.inverse() * attach1;
                                        let joint = FixedConstraint::new(
                                            BodyHandle::ground(),
                                            body,
                                            attach1,
                                            attach2,
                                        );
                                        grabbed_object_plane = (attach1_pos, -ray.dir);
                                        grabbed_object_joint =
                                            Some(self.world.add_constraint(joint));
                                        n.select()
                                    }
                                }
                            }

                            event.inhibited = true;
                        }
                    }
                    WindowEvent::MouseButton(_, Action::Release, _) => {
                        let mut graphics = self.graphics.borrow_mut();
                        if let Some(body) = grabbed_object {
                            for n in graphics
                                .body_nodes_mut(&self.world, body)
                                .unwrap()
                                .iter_mut()
                            {
                                n.unselect()
                            }
                        }

                        if let Some(joint) = grabbed_object_joint {
                            let _ = self.world.remove_constraint(joint);
                        }

                        grabbed_object = None;
                        grabbed_object_joint = None;
                    }
                    WindowEvent::CursorPos(x, y) => {
                        cursor_pos.x = x as f32;
                        cursor_pos.y = y as f32;

                        // update the joint
                        if let Some(joint) = grabbed_object_joint {
                            let size = self.window.size();
                            let (pos, dir) = self.graphics
                                .borrow()
                                .camera()
                                .unproject(&cursor_pos, &size);
                            let (ref ppos, ref pdir) = grabbed_object_plane;

                            if let Some(inter) = query::ray_internal::plane_toi_with_ray(
                                ppos,
                                pdir,
                                &Ray::new(pos, dir),
                            ) {
                                let joint = self.world
                                    .constraint_mut(joint)
                                    .downcast_mut::<FixedConstraint<f32>>()
                                    .unwrap();
                                joint.set_anchor_1(Isometry3::new(
                                    (pos + dir * inter).coords,
                                    na::zero(),
                                ))
                            }
                        }

                        event.inhibited = self.window.glfw_window().get_key(Key::RightShift)
                            != Action::Release
                            || self.window.glfw_window().get_key(Key::LeftShift) != Action::Release
                            || self.window.glfw_window().get_key(Key::RightControl)
                                != Action::Release
                            || self.window.glfw_window().get_key(Key::LeftControl)
                                != Action::Release;
                    }
                    //         WindowEvent::Key(Key::Tab, _, Action::Release, _) => self.graphics.borrow_mut().switch_cameras(),
                    WindowEvent::Key(Key::T, _, Action::Release, _) => {
                        if running == RunMode::Stop {
                            running = RunMode::Running;
                        } else {
                            running = RunMode::Stop;
                        }
                    }
                    WindowEvent::Key(Key::S, _, Action::Release, _) => running = RunMode::Step,
                    //         WindowEvent::Key(Key::B, _, Action::Release, _) => {
                    //             // XXX: there is a bug on kiss3d with the removal of objects.
                    //             // draw_aabbs = !draw_aabbs;
                    //             // if draw_aabbs {
                    //             //     graphics.enable_aabb_draw(&mut self.window);
                    //             // }
                    //             // else {
                    //             //     graphics.disable_aabb_draw(&mut self.window);
                    //             // }
                    //         },
                    WindowEvent::Key(Key::Space, _, Action::Release, _) => {
                        let mut graphics = self.graphics.borrow_mut();
                        draw_colls = !draw_colls;
                        for co in self.world.colliders() {
                            // FIXME: ugly clone.
                            if let Some(ns) = graphics.body_nodes_mut(&self.world, co.data().body())
                            {
                                for n in ns.iter_mut() {
                                    if draw_colls {
                                        n.scene_node_mut().set_lines_width(1.0);
                                        n.scene_node_mut().set_surface_rendering_activation(false);
                                    } else {
                                        n.scene_node_mut().set_lines_width(0.0);
                                        n.scene_node_mut().set_surface_rendering_activation(true);
                                    }
                                }
                            }
                        }
                    }
                    //      WindowEvent::Key(Key::Num1, _, Action::Press, _) => {
            //          let mut graphics = self.graphics.borrow_mut();
            //          let geom   = Ball::new(0.5f32);
            //          let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

            //          let cam_transfom;

            //          {
            //              let cam      = graphics.camera();
            //              cam_transfom = cam.view_transform().inverse()
            //          }

            //          rb.append_translation(&cam_transfom.translation);

            //          let front = cam_transfom.rotation * -Vector3::z();

            //          rb.set_lin_vel(front * 40.0f32);

            //          let body = self.world.add_rigid_body(rb);
            //          graphics.add(&mut self.window, body, &self.world.rigid_bodies());
            //      },
            //      WindowEvent::Key(Key::Num2, _, Action::Press, _) => {
            //          let mut graphics = self.graphics.borrow_mut();
            //          let geom   = Cuboid::new(Vector3::new(0.5f32, 0.5, 0.5));
            //          let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

            //          let cam_transform;

            //          {
            //              let cam = graphics.camera();
            //              cam_transform = cam.view_transform().inverse()
            //          }

            //          rb.append_translation(&cam_transform.translation);

            //          let front = cam_transform.rotation * -Vector3::z();

            //          rb.set_lin_vel(front * 40.0f32);

            //          let body = self.world.add_rigid_body(rb);
            //          graphics.add(&mut self.window, body, &self.world.rigid_bodies());
            //      }
                    _ => {}
                }
            }

            if running != RunMode::Stop {
                let before = time::precise_time_s();
                for _ in 0..self.nsteps {
                    for f in &self.callbacks {
                        f(&mut self.world, self.time)
                    }
                    self.world.step();
                    if !self.hide_counters {
                        println!("{}", self.world.performance_counters());
                    }
                    self.time += self.world.timestep();
                }
                self.physics_timer = time::precise_time_s() - before;

                self.graphics.borrow_mut().draw(&self.world);
            }

            if draw_colls {
                // self.graphics.borrow_mut().draw_positions(&mut self.window, &self.world.rigid_bodies());
                draw_collisions(
                    &mut self.window,
                    &mut self.world,
                    &mut self.persistant_contacts,
                    running != RunMode::Stop,
                );
            }

            if running == RunMode::Step {
                running = RunMode::Stop;
            }

            let color = Point3::new(0.0, 0.0, 0.0);

            if true {
                //running != RunMode::Stop {
                self.window.draw_text(
                    &format!(
                        "Time: {:.*}sec.",
                        4,
                        self.world.performance_counters().step_time()
                    )[..],
                    &Point2::origin(),
                    &font,
                    &color,
                );
            } else {
                self.window
                    .draw_text("Paused", &Point2::origin(), &font, &color);
            }

            self.window
                .render_with_camera(self.graphics.borrow_mut().camera_mut());
        }
    }
}

fn draw_collisions(
    window: &mut Window,
    world: &World<f32>,
    existing: &mut HashMap<GenerationalId, bool>,
    running: bool,
) {
    for (_, _, manifold) in world.collision_world().contact_manifolds() {
        for c in manifold.contacts() {
            if existing.contains_key(&c.id) {
                if running {
                    existing.insert(c.id, true);
                }
            } else {
                existing.insert(c.id, false);
            }

            let color = if existing[&c.id] {
                Point3::new(0.0, 0.0, 1.0)
            } else {
                Point3::new(1.0, 0.0, 0.0)
            };

            window.draw_line(&c.contact.world1, &c.contact.world2, &color);
            // let center = na::center(&c.world1, &c.world2);
            // let end    = center + c.normal * 0.4f32;
            // window.draw_line(&center, &end, &Point3::new(0.0, 1.0, 1.0))
        }
    }
}
