use std::os;
use std::num::{Zero, One, Bounded};
use std::rc::Rc;
use std::cell::RefCell;
use time;
use glfw;
use nalgebra::na::{Vec2, Vec3, Translation, Iso3};
use nalgebra::na;
use kiss3d::window::Window;
use kiss3d::light;
use kiss3d::text::Font;
use kiss3d::loader::obj;
use ncollide::geom::{Cuboid, Ball};
use ncollide::ray;
use ncollide::ray::Ray;
use nphysics::detection::Detector;
use nphysics::detection::constraint::{RBRB, BallInSocketConstraint, FixedConstraint};
use nphysics::detection::joint::{Anchor, Fixed, Joint};
use nphysics::object::RigidBody;
use nphysics::world::World;
use engine::GraphicsManager;


fn usage(exe_name: &str) {
    println!("Usage: {:s} [OPTION] ", exe_name);
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
    println!("    CTRL + click + drag - select and drag an object using a ball-in-socket joint.");
    println!("    SHIFT + click - remove an object.");
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

    pub fn look_at(&mut self, eye: Vec3<f32>, at: Vec3<f32>) {
        self.graphics.look_at(eye, at);
    }

    pub fn set_color(&mut self, rb: &Rc<RefCell<RigidBody>>, color: Vec3<f32>) {
        self.graphics.set_color(rb, color);
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Vec3<f32>>, Vec<uint>)> {
        let path    = Path::new(path);
        let empty   = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").ok().expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().to_owned().unwrap();
            let indices  = m.faces().read().to_owned().unwrap();

            let mut flat_indices = Vec::new();

            for i in indices.into_iter() {
                flat_indices.push(i.x as uint);
                flat_indices.push(i.y as uint);
                flat_indices.push(i.z as uint);
            }

            let m = (vertices, flat_indices);

            res.push(m);
        }

        res
    }

    pub fn run(&mut self) {
        let args        = os::args();
        let mut running = Running;

        if args.len() > 1 {
            for arg in args.iter() {
                if arg.as_slice() == "--help" || arg.as_slice() == "-h" {
                    usage(args[0].as_slice());
                    os::set_exit_status(1);
                    return;
                }
                else if arg.as_slice() == "--pause" {
                    running = Stop;
                }
            }
        }

        let font_mem       = include_bin!("Inconsolata.otf");
        let font           = Font::from_memory(font_mem, 60);
        let mut draw_colls = false;

        let mut cursor_pos = Vec2::new(0.0f32, 0.0);
        let mut grabbed_object: Option<Rc<RefCell<RigidBody>>> = None;
        let mut grabbed_object_joint: Option<Rc<RefCell<Fixed>>> = None;
        let mut grabbed_object_plane: (Vec3<f32>, Vec3<f32>) = (Zero::zero(), Zero::zero());


        self.window.set_framerate_limit(Some(60));
        self.window.set_light(light::StickToCamera);


        while !self.window.should_close() {
            for mut event in self.window.events().iter() {
                match event.value {
                    glfw::MouseButtonEvent(_, glfw::Press, modifier) => {
                        if modifier.contains(glfw::Shift) {
                            // XXX: huge and uggly code duplication
                            let size = self.window.size();
                            let (pos, dir) = self.graphics.camera().unproject(&cursor_pos, &size);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut interferences = Vec::new();
                            self.world.cast_ray(&ray, &mut interferences);

                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            for (b, toi) in interferences.into_iter() {
                                if toi < mintoi {
                                    mintoi = toi;
                                    minb   = Some(b);
                                }
                            }

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
                            let mut interferences = Vec::new();
                            self.world.cast_ray(&ray, &mut interferences);

                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            for (b, toi) in interferences.into_iter() {
                                if toi < mintoi {
                                    mintoi = toi;
                                    minb   = Some(b);
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
                                    for sn in self.graphics.body_to_scene_node(b).unwrap().iter_mut() {
                                        match grabbed_object_joint {
                                            Some(ref j) => self.world.remove_fixed(j),
                                            None        => { }
                                        }

                                        let _1: Iso3<f32> = One::one();
                                        let attach2 = na::append_translation(&_1, &(ray.orig + ray.dir * mintoi));
                                        let attach1 = na::inv(&na::transformation(b.borrow().transform_ref())).unwrap() * attach2;
                                        let anchor1 = Anchor::new(Some(minb.as_ref().unwrap().clone()), attach1);
                                        let anchor2 = Anchor::new(None, attach2);
                                        let joint   = Fixed::new(anchor1, anchor2);
                                        grabbed_object_plane = (na::translation(&attach2), -ray.dir);
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
                    glfw::MouseButtonEvent(_, glfw::Release, _) => {
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
                    glfw::CursorPosEvent(x, y) => {
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
                                        let _1: Iso3<f32> = One::one();
                                        j.borrow_mut().set_local2(na::append_translation(&_1, &(pos + dir * inter)))
                                    },
                                    None => { }
                                }

                            },
                            None => { }
                        }

                        event.inhibited =
                            self.window.glfw_window().get_key(glfw::KeyRightShift)   != glfw::Release ||
                            self.window.glfw_window().get_key(glfw::KeyLeftShift)    != glfw::Release ||
                            self.window.glfw_window().get_key(glfw::KeyRightControl) != glfw::Release ||
                            self.window.glfw_window().get_key(glfw::KeyLeftControl)  != glfw::Release;
                    },
                    glfw::KeyEvent(glfw::KeyTab, _, glfw::Release, _) => self.graphics.switch_cameras(),
                    glfw::KeyEvent(glfw::KeyT, _, glfw::Release, _) => {
                        if running == Stop {
                            running = Running;
                        }
                        else {
                            running = Stop;
                        }
                    },
                    glfw::KeyEvent(glfw::KeyS, _, glfw::Release, _) => running = Step,
                    glfw::KeyEvent(glfw::KeyB, _, glfw::Release, _) => {
                        // XXX: there is a bug on kiss3d with the removal of objects.
                        // draw_aabbs = !draw_aabbs;
                        // if draw_aabbs {
                        //     graphics.enable_aabb_draw(&mut self.window);
                        // }
                        // else {
                        //     graphics.disable_aabb_draw(&mut self.window);
                        // }
                    },
                    glfw::KeyEvent(glfw::KeySpace, _, glfw::Release, _) => {
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
                    glfw::KeyEvent(glfw::Key1, _, glfw::Press, _) => {
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
                    glfw::KeyEvent(glfw::Key2, _, glfw::Press, _) => {
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
                    },
                    glfw::KeyEvent(glfw::Key3, _, glfw::Press, _) => {
                        let geom   = Cuboid::new(Vec3::new(0.5f32, 0.5f32, 0.5f32));
                        let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                        let cam_transfom;

                        {
                            let cam = self.graphics.camera();
                            cam_transfom = cam.view_transform();
                        }

                        rb.append_translation(&na::translation(&cam_transfom));

                        let front = na::rotate(&cam_transfom, &Vec3::z());

                        rb.set_lin_vel(front * 400.0f32);

                        let body = self.world.add_body(rb);
                        // physics.add_ccd_to(body, 0.4, 1.0);
                        self.graphics.add(&mut self.window, body);
                        fail!("FIXME: review ccd");
                    },
                    _ => { }
                }
            }

            let before = time::precise_time_s();

            if running != Stop {
                self.world.step(0.016);
                self.graphics.draw();
            }

            if running == Step {
                running = Stop;
            }

            if draw_colls {
                self.graphics.draw_positions(&mut self.window);
                draw_collisions(&mut self.window, &mut self.world);
            }

            let color = Vec3::new(1.0, 1.0, 1.0);

            if running != Stop {
                let dt = time::precise_time_s() - before;

                self.window.draw_text(dt.to_string().as_slice(), &na::zero(), &font, &color);
            }
            else {
                self.window.draw_text("Paused", &na::zero(), &font, &color);
            }

            self.window.render_with_camera(self.graphics.camera());
        }
    }
}

#[deriving(PartialEq)]
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
            RBRB(_, _, ref c) => {
                window.draw_line(&c.world1, &c.world2, &Vec3::x());

                let center = (c.world1 + c.world2) / 2.0f32;
                let end    = center + c.normal * 0.4f32;
                window.draw_line(&center, &end, &Vec3::new(0.0, 1.0, 1.0))
            },
            BallInSocketConstraint(ref bis) => {
                let bbis = bis.borrow();
                window.draw_line(&bbis.anchor1_pos(), &bbis.anchor2_pos(), &Vec3::y());
            },
            FixedConstraint(ref f) => {
                // FIXME: draw the rotation too
                window.draw_line(&na::translation(&f.borrow().anchor1_pos()), &na::translation(&f.borrow().anchor2_pos()), &Vec3::y());
            }
        }
    }
}
