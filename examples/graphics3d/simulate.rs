use std::os;
use std::num::{Zero, One, Bounded};
use std::rc::Rc;
use std::cell::RefCell;
use time;
use glfw;
use nalgebra::na::{Vec2, Vec3, Translation, Iso3};
use nalgebra::na;
use kiss3d::window::Window;
use kiss3d::window;
use kiss3d::light;
use kiss3d::text::Font;
use ncollide::geom::{Box, Ball};
use ncollide::ray;
use ncollide::ray::Ray;
use nphysics::detection::Detector;
use nphysics::detection::constraint::{RBRB, BallInSocket, Fixed};
use nphysics::detection::joint::{Anchor, Fixed};
use nphysics::object::{RigidBody, Dynamic};
use nphysics::world::World;
use engine::GraphicsManager;

fn usage(exe_name: &str) {
    println!("Usage: {:s}", exe_name);
    println!("The following keyboard commands are supported:");
    println!("    t      - pause/continue the simulation.");
    println!("    s      - pause then execute only one simulation step.");
    println!("    r      - show/hide a ray centered on the camera, directed toward the camera front axis.");
    println!("    1      - launch a ball.");
    println!("    2      - launch a cube.");
    println!("    3      - launch a fast cube using continuous collision detection.");
    println!("    TAB    - switch camera mode (first-person or arc-ball).");
    println!("    CTRL + click + drag - select and drag an object using a ball-in-socket joint.");
    println!("    SHIFT + click - remove an object.");
    println!("    arrows - move around when in first-person camera mode.");
    println!("    space  - switch wireframe mode. When ON, the contacts points and normals are displayed.");
}

pub fn simulate(builder: |&mut Window, &mut GraphicsManager| -> World) {
    let args = os::args();

    if args.len() > 1 {
        usage(args[0]);
        os::set_exit_status(1);
        return;
    }

    Window::spawn("nphysics: 3d demo", |window| {
        let font           = Font::new(&Path::new("Inconsolata.otf"), 60);
        let mut running    = Running;
        let mut draw_colls = false;
        let mut graphics   = GraphicsManager::new();
        graphics.init_camera(window);
        let mut physics    = builder(window, &mut graphics);

        let mut ray_to_draw = None;

        let mut cursor_pos = Vec2::new(0.0f32, 0.0);
        let mut grabbed_object: Option<Rc<RefCell<RigidBody>>> = None;
        let mut grabbed_object_joint: Option<Rc<RefCell<Fixed>>> = None;
        let mut grabbed_object_plane: (Vec3<f32>, Vec3<f32>) = (Zero::zero(), Zero::zero());


        window.set_framerate_limit(Some(60));
        window.set_light(light::StickToCamera);

        window.render_loop(|w| {
            w.poll_events(|w, event| {
                match *event {
                    glfw::MouseButtonEvent(_, glfw::Press, modifier) => {
                        if modifier.contains(glfw::Shift) {
                            // XXX: huge and uggly code duplication
                            let (pos, dir) = w.unproject(&cursor_pos);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut interferences = Vec::new();
                            physics.cast_ray(&ray, &mut interferences);

                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            for (b, toi) in interferences.move_iter() {
                                if toi < mintoi {
                                    mintoi = toi;
                                    minb   = Some(b);
                                }
                            }

                            if minb.is_some() {
                                let b = minb.as_ref().unwrap();
                                if b.borrow().can_move() {
                                    physics.remove_body(b);
                                    graphics.remove(w, b);
                                }
                            }

                            false
                        }
                        else if modifier.contains(glfw::Control) {
                            match grabbed_object {
                                Some(ref rb) => {
                                    for sn in graphics.body_to_scene_node(rb).unwrap().mut_iter() {
                                        sn.unselect()
                                    }
                                },
                                None => { }
                            }

                            // XXX: huge and uggly code duplication
                            let (pos, dir) = w.unproject(&cursor_pos);
                            let ray = Ray::new(pos, dir);

                            // cast the ray
                            let mut interferences = Vec::new();
                            physics.cast_ray(&ray, &mut interferences);

                            let mut mintoi = Bounded::max_value();
                            let mut minb   = None;

                            for (b, toi) in interferences.move_iter() {
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
                                    for sn in graphics.body_to_scene_node(b).unwrap().mut_iter() {
                                        match grabbed_object_joint {
                                            Some(ref j) => physics.remove_fixed(j),
                                            None    => { }
                                        }

                                        let _1: Iso3<f32> = One::one();
                                        let attach2 = na::append_translation(&_1, &(ray.orig + ray.dir * mintoi));
                                        let attach1 = na::inv(&na::transformation(b.borrow().transform_ref())).unwrap() * attach2;
                                        let anchor1 = Anchor::new(Some(minb.as_ref().unwrap().clone()), attach1);
                                        let anchor2 = Anchor::new(None, attach2);
                                        let joint   = Rc::new(RefCell::new(Fixed::new(anchor1, anchor2)));
                                        grabbed_object_joint = Some(joint.clone());
                                        grabbed_object_plane = (na::translation(&attach2), -ray.dir);
                                        physics.add_fixed(joint);
                                        // add a joint
                                        sn.select()
                                    }
                                },
                                None => { }
                            }

                            false
                        }
                        else {
                            true
                        }
                    },
                    glfw::MouseButtonEvent(_, glfw::Release, _) => {
                        match grabbed_object {
                            Some(ref b) => {
                                for sn in graphics.body_to_scene_node(b).unwrap().mut_iter() {
                                    sn.unselect()
                                }
                            },
                            None => { }
                        }

                        match grabbed_object_joint {
                            Some(ref j) => physics.remove_fixed(j),
                            None    => { }
                        }

                        grabbed_object       = None;
                        grabbed_object_joint = None;

                        true
                    },
                    glfw::CursorPosEvent(x, y) => {
                        cursor_pos.x = x as f32;
                        cursor_pos.y = y as f32;

                        // update the joint
                        match grabbed_object_joint {
                            Some(ref j) => {
                                let (pos, dir) = w.unproject(&cursor_pos);
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

                        w.glfw_window().get_key(glfw::KeyRightShift)       == glfw::Release &&
                            w.glfw_window().get_key(glfw::KeyLeftShift)    == glfw::Release &&
                            w.glfw_window().get_key(glfw::KeyRightControl) == glfw::Release &&
                            w.glfw_window().get_key(glfw::KeyLeftControl)  == glfw::Release
                    },
                    glfw::KeyEvent(glfw::KeyTab, _, glfw::Release, _) => {
                        graphics.switch_cameras(w);

                        true
                    },
                    glfw::KeyEvent(glfw::KeyT, _, glfw::Release, _) => {
                        if running == Stop {
                            running = Running;
                        }
                        else {
                            running = Stop;
                        }

                        true
                    },
                    glfw::KeyEvent(glfw::KeyS, _, glfw::Release, _) => {
                        running = Step;

                        true
                    },
                    glfw::KeyEvent(glfw::KeySpace, _, glfw::Release, _) => {
                        draw_colls = !draw_colls;
                        w.set_wireframe_mode(draw_colls);

                        true
                    },
                    glfw::KeyEvent(glfw::Key1, _, glfw::Press, _) => {
                        let geom   = Ball::new(0.5f32);
                        let mut rb = RigidBody::new(geom, 4.0f32, Dynamic, 0.3, 0.6);

                        let cam_transfom;

                        {
                            let cam      = w.camera();
                            cam_transfom = cam.view_transform();
                        }

                        rb.append_translation(&na::translation(&cam_transfom));

                        let front = na::rotate(&cam_transfom, &Vec3::z());

                        rb.set_lin_vel(front * 40.0f32);

                        let body = Rc::new(RefCell::new(rb));
                        physics.add_body(body.clone());
                        graphics.add(w, body);

                        true
                    },
                    glfw::KeyEvent(glfw::Key2, _, glfw::Press, _) => {
                        let geom   = Box::new(Vec3::new(0.5f32, 0.5, 0.5));
                        let mut rb = RigidBody::new(geom, 4.0f32, Dynamic, 0.3, 0.6);

                        let cam_transform;
                        
                        {
                            let cam = w.camera();
                            cam_transform = cam.view_transform();
                        }

                        rb.append_translation(&na::translation(&cam_transform));

                        let front = na::rotate(&cam_transform, &Vec3::z());

                        rb.set_lin_vel(front * 40.0f32);

                        let body = Rc::new(RefCell::new(rb));
                        physics.add_body(body.clone());
                        graphics.add(w, body);

                        true
                    },
                    glfw::KeyEvent(glfw::Key3, _, glfw::Press, _) => {
                        let geom   = Box::new(Vec3::new(0.5f32, 0.5f32, 0.5f32));
                        let mut rb = RigidBody::new(geom, 4.0f32, Dynamic, 0.3, 0.6);

                        let cam_transfom;
                        
                        {
                            let cam = w.camera();
                            cam_transfom = cam.view_transform();
                        }

                        rb.append_translation(&na::translation(&cam_transfom));

                        let front = na::rotate(&cam_transfom, &Vec3::z());

                        rb.set_lin_vel(front * 400.0f32);

                        let body = Rc::new(RefCell::new(rb));
                        physics.add_body(body.clone());
                        // physics.add_ccd_to(body, 0.4, 1.0);
                        graphics.add(w, body);
                        fail!("FIXME: review ccd");

                        true
                    },
                    glfw::KeyEvent(glfw::KeyR, _, glfw::Press, _) => {
                        if ray_to_draw.is_some() {
                            ray_to_draw = None;
                        }
                        else {
                            let cam_transform;
                            
                            {
                                let cam = w.camera();
                                cam_transform = cam.view_transform();
                            }

                            let pos           = na::translation(&cam_transform);
                            let front         = na::rotate(&cam_transform, &Vec3::z());

                            ray_to_draw = Some(Ray::new(pos, front));
                        }

                        true
                    },
                    _ => true
                }
            });

            let before = time::precise_time_s();

            if running != Stop {
                physics.step(0.016);
                graphics.draw();
            }

            if running == Step {
                running = Stop;
            }

            if draw_colls {
                draw_collisions(w, &mut physics);
            }

            match ray_to_draw {
                None          => { },
                Some(ref ray) => {
                    // cast a ray
                    let mut interferences = Vec::new();
                    physics.cast_ray(ray, &mut interferences);

                    let mut mintoi = Bounded::max_value();

                    for (_, toi) in interferences.move_iter() {
                        if toi < mintoi {
                            mintoi = toi;
                        }
                    }

                    w.draw_line(&ray.orig, &(ray.orig + ray.dir * mintoi), &Vec3::x())
                }
            }

            if running != Stop {
                let dt = time::precise_time_s() - before;
                w.draw_text(dt.to_str(), &na::zero(), &font, &Vec3::new(1.0, 1.0, 1.0));
            }
        })
    })
}

#[deriving(Eq)]
enum RunMode {
    Running,
    Stop,
    Step
}

fn draw_collisions(window: &mut window::Window, physics: &mut World) {
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
            BallInSocket(ref bis) => {
                let bbis = bis.borrow();
                window.draw_line(&bbis.anchor1_pos(), &bbis.anchor2_pos(), &Vec3::y());
            },
            Fixed(ref f) => {
                // FIXME: draw the rotation too
                window.draw_line(&na::translation(&f.borrow().anchor1_pos()), &na::translation(&f.borrow().anchor2_pos()), &Vec3::y());
            }
        }
    }
}
