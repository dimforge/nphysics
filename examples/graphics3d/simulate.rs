use std::os;
use std::num::{Zero, One, Bounded};
use std::rc::Rc;
use std::cell::RefCell;
use time;
use glfw;
use nalgebra::na::{Vec2, Vec3, Translation, Iso3};
use nalgebra::na;
use kiss3d::window::{Window, RenderFrame};
use kiss3d::light;
use kiss3d::text::Font;
use kiss3d::utils::Recorder;
use ncollide::geom::{Cuboid, Ball};
use ncollide::ray;
use ncollide::ray::Ray;
use nphysics::detection::Detector;
use nphysics::detection::constraint::{RBRB, BallInSocket, Fixed};
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
    println!("    r      - start/stop recording to the files {:s}_{{1,2,3,...}}.mpg.", exe_name);
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
    let args        = os::args();
    let mut running = Running;

    if args.len() > 1 {
        for arg in args.iter() {
            if arg.as_slice() == "--help" || arg.as_slice() == "-h" {
                usage(args.get(0).as_slice());
                os::set_exit_status(1);
                return;
            }
            else if arg.as_slice() == "--pause" {
                running = Stop;
            }
        }
    }

    let mut window = Window::new("nphysics: 3d demo");

    let mut recorder   = None;
    let mut irec       = 1u;
    let font           = Font::new(&Path::new("Inconsolata.otf"), 60);
    let mut draw_colls = false;
    let mut graphics   = GraphicsManager::new();
    let mut physics    = builder(&mut window, &mut graphics);

    let mut cursor_pos = Vec2::new(0.0f32, 0.0);
    let mut grabbed_object: Option<Rc<RefCell<RigidBody>>> = None;
    let mut grabbed_object_joint: Option<Rc<RefCell<Fixed>>> = None;
    let mut grabbed_object_plane: (Vec3<f32>, Vec3<f32>) = (Zero::zero(), Zero::zero());


    window.set_framerate_limit(Some(60));
    window.set_light(light::StickToCamera);


    for mut frame in window.iter() {
        for mut event in frame.events().iter() {
            match event.value {
                glfw::MouseButtonEvent(_, glfw::Press, modifier) => {
                    if modifier.contains(glfw::Shift) {
                        // XXX: huge and uggly code duplication
                        let size = frame.window().size();
                        let (pos, dir) = graphics.camera().unproject(&cursor_pos, &size);
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
                                graphics.remove(frame.window(), b);
                            }
                        }

                        event.inhibited = true;
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
                        let size = frame.window().size();
                        let (pos, dir) = graphics.camera().unproject(&cursor_pos, &size);
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

                        event.inhibited = true;
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
                },
                glfw::CursorPosEvent(x, y) => {
                    cursor_pos.x = x as f32;
                    cursor_pos.y = y as f32;

                    // update the joint
                    match grabbed_object_joint {
                        Some(ref j) => {
                            let size = frame.window().size();
                            let (pos, dir) = graphics.camera().unproject(&cursor_pos, &size);
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
                        frame.window().glfw_window().get_key(glfw::KeyRightShift)   != glfw::Release ||
                        frame.window().glfw_window().get_key(glfw::KeyLeftShift)    != glfw::Release ||
                        frame.window().glfw_window().get_key(glfw::KeyRightControl) != glfw::Release ||
                        frame.window().glfw_window().get_key(glfw::KeyLeftControl)  != glfw::Release;
                },
                glfw::KeyEvent(glfw::KeyR, _, glfw::Press, _) => {
                    if recorder.is_some() {
                        recorder = None;
                    }
                    else {
                        let rec = Recorder::new(Path::new(format!("{:s}_{}.mpg", *args.get(0), irec)),
                        frame.window().width()  as uint,
                        frame.window().height() as uint);
                        recorder = Some(rec);
                        irec = irec + 1;
                    }
                },
                glfw::KeyEvent(glfw::KeyTab, _, glfw::Release, _) => graphics.switch_cameras(),
                glfw::KeyEvent(glfw::KeyT, _, glfw::Release, _) => {
                    if running == Stop {
                        running = Running;
                    }
                    else {
                        running = Stop;
                    }
                },
                glfw::KeyEvent(glfw::KeyS, _, glfw::Release, _) => running = Step,
                glfw::KeyEvent(glfw::KeySpace, _, glfw::Release, _) => {
                    draw_colls = !draw_colls;
                    if draw_colls {
                        frame.window().scene_mut().set_lines_width(1.0);
                        frame.window().scene_mut().set_surface_rendering_activation(false);
                    }
                    else {
                        frame.window().scene_mut().set_lines_width(0.0);
                        frame.window().scene_mut().set_surface_rendering_activation(true);
                    }
                },
                glfw::KeyEvent(glfw::Key1, _, glfw::Press, _) => {
                    let geom   = Ball::new(0.5f32);
                    let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                    let cam_transfom;

                    {
                        let cam      = graphics.camera();
                        cam_transfom = cam.view_transform();
                    }

                    rb.append_translation(&na::translation(&cam_transfom));

                    let front = na::rotate(&cam_transfom, &Vec3::z());

                    rb.set_lin_vel(front * 40.0f32);

                    let body = Rc::new(RefCell::new(rb));
                    physics.add_body(body.clone());
                    graphics.add(frame.window(), body.clone());
                },
                glfw::KeyEvent(glfw::Key2, _, glfw::Press, _) => {
                    let geom   = Cuboid::new(Vec3::new(0.5f32, 0.5, 0.5));
                    let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                    let cam_transform;

                    {
                        let cam = graphics.camera();
                        cam_transform = cam.view_transform();
                    }

                    rb.append_translation(&na::translation(&cam_transform));

                    let front = na::rotate(&cam_transform, &Vec3::z());

                    rb.set_lin_vel(front * 40.0f32);

                    let body = Rc::new(RefCell::new(rb));
                    physics.add_body(body.clone());
                    graphics.add(frame.window(), body.clone());
                },
                glfw::KeyEvent(glfw::Key3, _, glfw::Press, _) => {
                    let geom   = Cuboid::new(Vec3::new(0.5f32, 0.5f32, 0.5f32));
                    let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                    let cam_transfom;

                    {
                        let cam = graphics.camera();
                        cam_transfom = cam.view_transform();
                    }

                    rb.append_translation(&na::translation(&cam_transfom));

                    let front = na::rotate(&cam_transfom, &Vec3::z());

                    rb.set_lin_vel(front * 400.0f32);

                    let body = Rc::new(RefCell::new(rb));
                    physics.add_body(body.clone());
                    // physics.add_ccd_to(body, 0.4, 1.0);
                    graphics.add(frame.window(), body);
                    fail!("FIXME: review ccd");
                },
                _ => { }
            }
        }

        let before = time::precise_time_s();

        if running != Stop {
            physics.step(0.016);
            graphics.draw();
        }

        if running == Step {
            running = Stop;
        }

        if draw_colls {
            draw_collisions(&mut frame, &mut physics);
        }

        let _ = recorder.as_mut().map(|r| r.snap(frame.window()));

        let color = if recorder.is_none() { Vec3::new(1.0, 1.0, 1.0) } else { Vec3::x() };

        if running != Stop {
            let dt    = time::precise_time_s() - before;

            frame.draw_text(dt.to_str().as_slice(), &na::zero(), &font, &color);
        }
        else {
            frame.draw_text("Paused", &na::zero(), &font, &color);
        }

        frame.set_camera(graphics.camera());
    }
}

#[deriving(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step
}

fn draw_collisions(frame: &mut RenderFrame, physics: &mut World) {
    let mut collisions = Vec::new();

    physics.interferences(&mut collisions);

    for c in collisions.iter() {
        match *c {
            RBRB(_, _, ref c) => {
                frame.draw_line(&c.world1, &c.world2, &Vec3::x());

                let center = (c.world1 + c.world2) / 2.0f32;
                let end    = center + c.normal * 0.4f32;
                frame.draw_line(&center, &end, &Vec3::new(0.0, 1.0, 1.0))
            },
            BallInSocket(ref bis) => {
                let bbis = bis.borrow();
                frame.draw_line(&bbis.anchor1_pos(), &bbis.anchor2_pos(), &Vec3::y());
            },
            Fixed(ref f) => {
                // FIXME: draw the rotation too
                frame.draw_line(&na::translation(&f.borrow().anchor1_pos()), &na::translation(&f.borrow().anchor2_pos()), &Vec3::y());
            }
        }
    }
}
