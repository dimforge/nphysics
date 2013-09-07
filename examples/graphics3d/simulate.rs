use std::os;
use std::num::Zero;
use extra::time;
use glfw;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::Transform;
use nalgebra::vec::{Vec2, Vec3};
use kiss3d::window::Window;
use kiss3d::window;
use kiss3d::event;
use ncollide::geom::{Geom, Ball, Box};
use ncollide::ray;
use ncollide::ray::Ray;
use nphysics::aliases::dim3;
use nphysics::detection::constraint::{RBRB, BallInSocket};
use nphysics::detection::joint::ball_in_socket::BallInSocket;
use nphysics::detection::joint::anchor::Anchor;
use nphysics::object::{RigidBody, Dynamic, RB};
use engine::{SceneNode, GraphicsManager};

fn usage(exe_name: &str) {
    println("Usage: " + exe_name);
    println("The following keyboard commands are supported:");
    println("    t      - pause/continue the simulation.");
    println("    s      - pause then execute only one simulation step.");
    println("    r      - show/hide a ray centered on the camera, directed toward the camera front axis.");
    println("    1      - launch a ball.");
    println("    2      - launch a cube.");
    println("    3      - launch a fast cube using continuous collision detection.");
    println("    TAB    - switch camera mode (first-person or arc-ball).");
    println("    CTRL + click + drag - select and drag an object using a ball-in-socket joint.");
    println("    SHIFT + click - remove an object.");
    println("    arrows - move around when in first-person camera mode.");
    println("    space  - switch wireframe mode. When ON, the contacts points and normals are displayed.");
}

pub fn simulate(builder: ~fn(&mut Window, &mut GraphicsManager) -> dim3::BodyWorld3d<f64>) {
    let args = os::args();

    if args.len() > 1 {
        usage(args[0]);
        os::set_exit_status(1);
        return;
    }

    do Window::spawn("nphysics: 3d demo") |window| {
        let running    = @mut Running;
        let draw_colls = @mut false;
        let graphics   = @mut GraphicsManager::new(window);
        let physics    = @mut builder(window, graphics);

        let ray_to_draw = @mut None;

        let cursor_pos = @mut Vec2::new(0.0f64, 0.0);
        let grabbed_object: @mut Option<@mut dim3::Body3d<f64>> = @mut None;
        let grabbed_object_joint: @mut Option<@mut dim3::BallInSocket3d<f64>> = @mut None;
        let grabbed_object_plane: @mut (Vec3<f64>, Vec3<f64>) = @mut (Zero::zero(), Zero::zero());

        do window.set_mouse_callback |w, event| {
            match *event {
                event::ButtonPressed(_, modifier) => {
                    if modifier.contains(glfw::Shift) {
                        // XXX: huge and uggly code duplication
                        let (pos, dir) = w.unproject(&*cursor_pos);
                        let ray = Ray::new(pos, dir);

                        // cast the ray
                        let mut interferences = ~[];
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
                            let b = minb.unwrap();
                            if b.can_move() {
                                physics.remove_body(b);
                                graphics.remove(w, b);
                            }
                        }

                        false
                    }
                    else if modifier.contains(glfw::Control) {
                        match *grabbed_object {
                            Some(rb) => {
                                for sn in graphics.body_to_scene_node(rb).unwrap().iter() {
                                    sn.unselect()
                                }
                            },
                            None => { }
                        }

                        // XXX: huge and uggly code duplication
                        let (pos, dir) = w.unproject(&*cursor_pos);
                        let ray = Ray::new(pos, dir);

                        // cast the ray
                        let mut interferences = ~[];
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
                            let b = minb.unwrap();
                            if b.can_move() {
                                *grabbed_object = Some(b)
                            }
                        }

                        match *grabbed_object {
                            Some(b) => {
                                for sn in graphics.body_to_scene_node(b).unwrap().iter() {
                                    match *grabbed_object_joint {
                                        Some(j) => physics.remove_ball_in_socket(j),
                                        None    => { }
                                    }

                                    let rb      = b.to_rigid_body_or_fail();
                                    let attach2 = ray.orig + ray.dir * mintoi;
                                    let attach1 = rb.transform_ref().inv_transform(&attach2);
                                    let anchor1 = Anchor::new(Some(minb.unwrap()), attach1);
                                    let anchor2 = Anchor::new(None, attach2);
                                    let joint   = @mut BallInSocket::new(anchor1, anchor2);
                                    *grabbed_object_joint = Some(joint);
                                    *grabbed_object_plane = (attach2, -ray.dir);
                                    physics.add_ball_in_socket(joint);
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
                event::ButtonReleased(_, _) => {
                    match *grabbed_object {
                        Some(b) => {
                            for sn in graphics.body_to_scene_node(b).unwrap().iter() {
                                sn.unselect()
                            }
                        },
                        None => { }
                    }

                    match *grabbed_object_joint {
                        Some(j) => physics.remove_ball_in_socket(j),
                        None    => { }
                    }

                    *grabbed_object       = None;
                    *grabbed_object_joint = None;

                    true
                },
                event::CursorPos(x, y) => {
                    cursor_pos.x = x as f64;
                    cursor_pos.y = y as f64;

                    // update the joint
                    match *grabbed_object_joint {
                        Some(j) => {
                            let (pos, dir) = w.unproject(&*cursor_pos);
                            let (ref ppos, ref pdir) = *grabbed_object_plane;

                            match ray::plane_toi_with_ray(ppos, pdir, &Ray::new(pos, dir)) {
                                Some(inter) =>
                                    j.set_local2(pos + dir * inter),
                                None => { }
                            }

                        },
                        None => { }
                    }

                    w.glfw_window().get_key(glfw::KEY_RIGHT_SHIFT)   == glfw::FALSE &&
                    w.glfw_window().get_key(glfw::KEY_LEFT_SHIFT)    == glfw::FALSE &&
                    w.glfw_window().get_key(glfw::KEY_RIGHT_CONTROL) == glfw::FALSE &&
                    w.glfw_window().get_key(glfw::KEY_LEFT_CONTROL)  == glfw::FALSE
                },
                _ => true
            }
        }

        do window.set_keyboard_callback |w, event| {
            match *event {
                event::KeyReleased(glfw::KEY_TAB) => {
                    graphics.switch_cameras(w)
                },
                event::KeyReleased(glfw::KEY_T) => {
                    if *running == Stop {
                        *running = Running;
                    }
                    else {
                        *running = Stop;
                    }
                },
                event::KeyReleased(glfw::KEY_S) => {
                    *running = Step
                },
                event::KeyReleased(glfw::KEY_SPACE) => {
                    *draw_colls = !*draw_colls;
                    w.set_wireframe_mode(*draw_colls);
                },
                event::KeyPressed(glfw::KEY_1) => {
                    let geom   = Geom::new_ball(Ball::new(0.5f64));
                    let mut rb = RigidBody::new(geom, 4.0f64, Dynamic, 0.3, 0.6);

                    let cam_transfom = w.camera().view_transform();
                    rb.translate_by(&cam_transfom.translation());

                    let front = cam_transfom.rotate(&Vec3::z());

                    rb.set_lin_vel(front * 40.0);

                    let body = @mut RB(rb);
                    physics.add_body(body);
                    graphics.add(w, body);
                },
                event::KeyPressed(glfw::KEY_2) => {
                    let geom   = Geom::new_box(Box::new(Vec3::new(0.5f64, 0.5, 0.5)));
                    let mut rb = RigidBody::new(geom, 4.0f64, Dynamic, 0.3, 0.6);

                    let cam_transform = w.camera().view_transform();
                    rb.translate_by(&cam_transform.translation());

                    let front = cam_transform.rotate(&Vec3::z());

                    rb.set_lin_vel(front * 40.0);

                    let body = @mut RB(rb);
                    physics.add_body(body);
                    graphics.add(w, body);
                },
                event::KeyPressed(glfw::KEY_3) => {
                    let geom   = Geom::new_box(Box::new(Vec3::new(0.5f64, 0.5f64, 0.5f64)));
                    let mut rb = RigidBody::new(geom, 4.0f64, Dynamic, 0.3, 0.6);

                    let cam_transfom = w.camera().view_transform();
                    rb.translate_by(&cam_transfom.translation());

                    let front = cam_transfom.rotate(&Vec3::z());

                    rb.set_lin_vel(front * 400.0);

                    let body = @mut RB(rb);
                    physics.add_body(body);
                    physics.add_ccd_to(body, 0.4, 1.0);
                    graphics.add(w, body);
                },
                event::KeyPressed(glfw::KEY_R) => {
                    if ray_to_draw.is_some() {
                        *ray_to_draw = None;
                    }
                    else {
                        let cam_transform = w.camera().view_transform();
                        let pos           = cam_transform.translation();
                        let front         = cam_transform.rotate(&Vec3::z());

                        *ray_to_draw = Some(Ray::new(pos, front));
                    }
                },
                _ => { }
            };

            true
        }

        window.set_framerate_limit(Some(60));
        window.set_light(window::StickToCamera);

        do window.render_loop |w| {
            let before = time::precise_time_s();

            if *running != Stop {
                physics.step(0.016);
                graphics.draw();
            }

            if *running == Step {
                *running = Stop;
            }

            if *draw_colls {
                draw_collisions(w, physics);
            }

            match *ray_to_draw {
                None          => { },
                Some(ref ray) => {
                    // cast a ray
                    let mut interferences = ~[];
                    physics.cast_ray(ray, &mut interferences);

                    let mut mintoi = Bounded::max_value();

                    for (_, toi) in interferences.move_iter() {
                        if toi < mintoi {
                            mintoi = toi;
                        }
                    }

                    let mintoi = 100.0;
                    w.draw_line(&ray.orig, &(ray.orig + ray.dir * mintoi), &Vec3::x())
                }
            }

            if *running != Stop {
                let dt = (time::precise_time_s() - before);
                println(dt.to_str() + "sec (" + (1.0 / dt).to_str() + " fps)");
            }
        }
    }
}

#[deriving(Eq)]
enum RunMode {
    Running,
    Stop,
    Step
}

fn draw_collisions(window: &mut window::Window, physics: &mut dim3::BodyWorld3d<f64>) {
    let mut collisions = ~[];

    for c in physics.world().detectors().iter() {
        c.interferences(&mut collisions);
    }

    for c in collisions.iter() {
        match *c {
            RBRB(_, _, c) => {
                window.draw_line(&c.world1, &c.world2, &Vec3::x());

                let center = (c.world1 + c.world2) / 2.0;
                let end    = center + c.normal * 0.4;
                window.draw_line(&center, &end, &Vec3::new(0.0, 1.0, 1.0))
            },
            BallInSocket(bis) => {
                window.draw_line(&bis.anchor1_pos(), &bis.anchor2_pos(), &Vec3::y());
            }
        }
    }
}
