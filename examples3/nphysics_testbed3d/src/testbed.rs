use std::env;
use std::rc::Rc;
use std::cell::RefCell;
use std::path::Path;
use time;
use glfw;
use glfw::{MouseButton, Key, Action, WindowEvent};
use na::{Pnt2, Pnt3, Vec3, Translation, Translate, Iso3, Bounded};
use na;
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::text::Font;
use kiss3d::loader::obj;
use ncollide::shape::{Cuboid, Ball};
use ncollide::ray;
use ncollide::ray::Ray;
use nphysics::detection::Detector;
use nphysics::detection::constraint::Constraint;
use nphysics::detection::joint::{Anchor, Fixed, Joint};
use nphysics::object::RigidBody;
use nphysics::world::World;
use engine::GraphicsManager;


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
    world:    World,
    window:   Window,
    graphics: GraphicsManager,
}

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let window   = Window::new("nphysics: 3d demo");

        Testbed {
            world:    World::new(),
            window:   window,
            graphics: graphics
        }
    }

    pub fn new(world: World) -> Testbed {
        let mut res = Testbed::new_empty();

        res.set_world(world);

        res
    }

    pub fn set_world(&mut self, world: World) {
        self.world = world;

        self.graphics.clear(&mut self.window);

        for rb in self.world.bodies() {
            self.graphics.add(&mut self.window, rb.clone());
        }
    }

    pub fn look_at(&mut self, eye: Pnt3<f32>, at: Pnt3<f32>) {
        self.graphics.look_at(eye, at);
    }

    pub fn set_color(&mut self, rb: &Rc<RefCell<RigidBody>>, color: Pnt3<f32>) {
        self.graphics.set_color(rb, color);
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Pnt3<f32>>, Vec<usize>)> {
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
                    env::set_exit_status(1);
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

        let mut cursor_pos = Pnt2::new(0.0f32, 0.0);
        let mut grabbed_object: Option<Rc<RefCell<RigidBody>>> = None;
        let mut grabbed_object_joint: Option<Rc<RefCell<Fixed>>> = None;
        let mut grabbed_object_plane: (Pnt3<f32>, Vec3<f32>) = (na::orig(), na::zero());


        self.window.set_framerate_limit(Some(60));
        self.window.set_light(Light::StickToCamera);


        while !self.window.should_close() {
            for mut event in self.window.events().iter() {
                match event.value {
                    WindowEvent::MouseButton(MouseButton::Button2, Action::Press, glfw::Control) => {
                        let geom   = Cuboid::new(Vec3::new(0.5f32, 0.5f32, 0.5f32));
                        let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                        let size = self.window.size();
                        let (pos, dir) = self.graphics.camera().unproject(&cursor_pos, &size);

                        rb.set_translation(pos.to_vec());
                        rb.set_lin_vel(dir * 1000.0f32);

                        let body = self.world.add_body(rb);
                        self.world.add_ccd_to(&body, 1.0);
                        self.graphics.add(&mut self.window, body);
                    },
                    WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                        if modifier.contains(glfw::Shift) {
                            // XXX: huge and uggly code duplication
                            let size = self.window.size();
                            let (pos, dir) = self.graphics.camera().unproject(&cursor_pos, &size);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            self.world.interferences_with_ray(&ray, |b, inter| {
                                if inter.toi < mintoi {
                                    mintoi = inter.toi;
                                    minb   = Some(b.clone());
                                }
                            });

                            if minb.is_some() {
                                let b = minb.as_ref().unwrap();
                                if b.borrow().can_move() {
                                    self.world.remove_body(b);
                                    self.graphics.remove(&mut self.window, b);
                                }
                            }

                            event.inhibited = true;
                        }
                        else if modifier.contains(glfw::Control) {
                            match grabbed_object {
                                Some(ref rb) => {
                                    for sn in self.graphics.body_to_scene_node(rb).unwrap().iter_mut() {
                                        sn.unselect()
                                    }
                                },
                                None => { }
                            }

                            // XXX: huge and uggly code duplication
                            let size = self.window.size();
                            let (pos, dir) = self.graphics.camera().unproject(&cursor_pos, &size);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            self.world.interferences_with_ray(&ray, |b, inter| {
                                if inter.toi < mintoi {
                                    mintoi = inter.toi;
                                    minb   = Some(b.clone());
                                }
                            });

                            if minb.is_some() {
                                let b = minb.as_ref().unwrap();
                                if b.borrow().can_move() {
                                    grabbed_object = Some(b.clone())
                                }
                            }

                            match grabbed_object {
                                Some(ref b) => {
                                    for sn in self.graphics.body_to_scene_node(b).unwrap().iter_mut() {
                                        match grabbed_object_joint {
                                            Some(ref j) => self.world.remove_fixed(j),
                                            None        => { }
                                        }

                                        let _1: Iso3<f32> = na::one();
                                        let attach2 = na::append_translation(&_1, (ray.orig + ray.dir * mintoi).as_vec());
                                        let attach1 = na::inv(&na::transformation(b.borrow().position())).unwrap() * attach2;
                                        let anchor1 = Anchor::new(Some(minb.as_ref().unwrap().clone()), attach1);
                                        let anchor2 = Anchor::new(None, attach2);
                                        let joint   = Fixed::new(anchor1, anchor2);
                                        grabbed_object_plane = (attach2.translate(&na::orig()), -ray.dir);
                                        grabbed_object_joint = Some(self.world.add_fixed(joint));
                                        // add a joint
                                        sn.select()
                                    }
                                },
                                None => { }
                            }

                            event.inhibited = true;
                        }
                    },
                    WindowEvent::MouseButton(_, Action::Release, _) => {
                        match grabbed_object {
                            Some(ref b) => {
                                for sn in self.graphics.body_to_scene_node(b).unwrap().iter_mut() {
                                    sn.unselect()
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
                                let (pos, dir) = self.graphics.camera().unproject(&cursor_pos, &size);
                                let (ref ppos, ref pdir) = grabbed_object_plane;

                                match ray::plane_toi_with_ray(ppos, pdir, &Ray::new(pos, dir)) {
                                    Some(inter) => {
                                        let _1: Iso3<f32> = na::one();
                                        j.borrow_mut().set_local2(na::append_translation(&_1, (pos + dir * inter).as_vec()))
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
                    WindowEvent::Key(Key::Tab, _, Action::Release, _) => self.graphics.switch_cameras(),
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
                        draw_colls = !draw_colls;
                        if draw_colls {
                            self.window.scene_mut().set_lines_width(1.0);
                            self.window.scene_mut().set_surface_rendering_activation(false);
                        }
                        else {
                            self.window.scene_mut().set_lines_width(0.0);
                            self.window.scene_mut().set_surface_rendering_activation(true);
                        }
                    },
                    WindowEvent::Key(Key::Num1, _, Action::Press, _) => {
                        let geom   = Ball::new(0.5f32);
                        let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                        let cam_transfom;

                        {
                            let cam      = self.graphics.camera();
                            cam_transfom = cam.view_transform();
                        }

                        rb.append_translation(&na::translation(&cam_transfom));

                        let front = na::rotate(&cam_transfom, &Vec3::z());

                        rb.set_lin_vel(front * 40.0f32);

                        let body = self.world.add_body(rb);
                        self.graphics.add(&mut self.window, body.clone());
                    },
                    WindowEvent::Key(Key::Num2, _, Action::Press, _) => {
                        let geom   = Cuboid::new(Vec3::new(0.5f32, 0.5, 0.5));
                        let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                        let cam_transform;

                        {
                            let cam = self.graphics.camera();
                            cam_transform = cam.view_transform();
                        }

                        rb.append_translation(&na::translation(&cam_transform));

                        let front = na::rotate(&cam_transform, &Vec3::z());

                        rb.set_lin_vel(front * 40.0f32);

                        let body = self.world.add_body(rb);
                        self.graphics.add(&mut self.window, body.clone());
                    }
                    _ => { }
                }
            }

            let dt;

            if running != RunMode::Stop {
                let before = time::precise_time_s();
                self.world.step(0.016);
                dt = time::precise_time_s() - before;

                self.graphics.draw();
            }
            else {
                dt = 0.0;
            }

            if running == RunMode::Step {
                running = RunMode::Stop;
            }

            if draw_colls {
                self.graphics.draw_positions(&mut self.window);
                draw_collisions(&mut self.window, &mut self.world);
            }

            let color = Pnt3::new(1.0, 1.0, 1.0);

            if running != RunMode::Stop {
                self.window.draw_text(&dt.to_string()[..], &na::orig(), &font, &color);
            }
            else {
                self.window.draw_text("Paused", &na::orig(), &font, &color);
            }

            self.window.render_with_camera(self.graphics.camera());
        }
    }
}

#[derive(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step
}

fn draw_collisions(window: &mut Window, physics: &mut World) {
    let mut collisions = Vec::new();

    physics.interferences(&mut collisions);

    for c in collisions.iter() {
        match *c {
            Constraint::RBRB(_, _, ref c) => {
                window.draw_line(&c.world1, &c.world2, &Pnt3::new(1.0, 0.0, 0.0));

                let center = na::center(&c.world1, &c.world2);
                let end    = center + c.normal * 0.4f32;
                window.draw_line(&center, &end, &Pnt3::new(0.0, 1.0, 1.0))
            },
            Constraint::BallInSocket(ref bis) => {
                let bbis = bis.borrow();
                window.draw_line(&bbis.anchor1_pos(), &bbis.anchor2_pos(), &Pnt3::new(0.0, 1.0, 0.0));
            },
            Constraint::Fixed(ref f) => {
                // FIXME: draw the rotation too
                window.draw_line(&f.borrow().anchor1_pos().translate(&na::orig()), &f.borrow().anchor2_pos().translate(&na::orig()), &Pnt3::new(0.0, 1.0, 0.0));
            }
        }
    }
}
