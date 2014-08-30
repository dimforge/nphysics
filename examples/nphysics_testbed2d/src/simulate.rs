use std::rc::Rc;
use std::cell::RefCell;
use std::os;
use rsfml::graphics::{RenderWindow, Font};
use rsfml::window::{ContextSettings, VideoMode, Close};
use rsfml::window::event;
use rsfml::window::keyboard;
use rsfml::graphics::Color;
use nalgebra::na::Vec2;
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics::detection::joint::BallInSocket;
use camera::Camera;
use fps::Fps;
use engine::GraphicsManager;
use draw_helper;

fn usage(exe_name: &str) {
    println!("Usage: {:s} [OPTION] ", exe_name);
    println!("");
    println!("Options:");
    println!("    --help  - prints this help message and exits.");
    println!("    --pause - do not start the simulation right away.");
    println!("");
    println!("The following keyboard commands are supported:");
    println!("    t     - pause/continue the simulation.");
    println!("    s     - pause then execute only one simulation step.");
    println!("    space - display/hide contacts.");
    println!("    CTRL + click + drag - select and drag an object using a ball-in-socket joint.");
}

pub fn simulate(builder: |&mut GraphicsManager| -> World) {
    let args        = os::args();
    let mut running = Running;

    if args.len() > 1 {
        if args.len() > 2 || args.get(1).as_slice() != "--pause" {
            usage(args.get(0).as_slice());
            os::set_exit_status(1);
            return;
        }
        else {
            running = Stop;
        }
    }

    let mut draw_colls = false;

    let mode    = VideoMode::new_init(800, 600, 32);
    let setting = ContextSettings {
        depth_bits:         10,
        stencil_bits:       10,
        antialiasing_level: 2,
        major_version:      0,
        minor_version:      1
    };
    let mut rwindow =
        match RenderWindow::new(mode, "nphysics demo", Close, &setting) {
            Some(rwindow) => rwindow,
            None => fail!("Error on creating window")
        };

    let mut camera = Camera::new();

    rwindow.set_framerate_limit(60);


    let mut graphics = GraphicsManager::new();
    let mut physics  = builder(&mut graphics);
    let     fnt      = Font::new_from_file("Inconsolata.otf").unwrap();
    let mut fps      = Fps::new(&fnt);
    let mut cursor_pos;
    let grabbed_object: Option<Rc<RefCell<RigidBody>>> = None;
    let grabbed_object_joint: Option<Rc<RefCell<BallInSocket>>> = None;

    while rwindow.is_open() {
        loop {
            match rwindow.poll_event() {
                event::KeyPressed{code, ..} => {
                    match code {
                        keyboard::Escape => rwindow.close(),
                        keyboard::S      => running = Step,
                        keyboard::Space  => draw_colls = !draw_colls,
                        keyboard::T      => {
                            if running == Stop {
                                running = Running;
                            }
                            else {
                                running = Stop;
                            }
                        },
                        _                => { }
                    }
                },
                event::MouseMoved{x, y} => {
                    cursor_pos = Vec2::new(x as f32, y as f32);
                    match grabbed_object {
                        Some(_) => {
                            let joint = grabbed_object_joint.as_ref().unwrap();
                            joint.borrow_mut().set_local2(cursor_pos);
                        },
                        None => camera.handle_event(&event::MouseMoved{x: x, y: y})
                    };
                },
                event::Closed  => rwindow.close(),
                event::NoEvent => break,
                e              => camera.handle_event(&e)
            }
        }

        rwindow.clear(&Color::black());

        fps.reset();

        if running != Stop {
            physics.step(0.016);
        }

        if running == Step {
            running = Stop;
        }
        fps.register_delta();
        graphics.draw(&mut rwindow, &camera);

        camera.activate_scene(&mut rwindow);
        if draw_colls {
            draw_helper::draw_colls(&rwindow, &mut physics);
        }

        camera.activate_ui(&mut rwindow);
        fps.draw_registered(&mut rwindow);

        rwindow.display();
    }

    rwindow.close();
}

#[deriving(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step
}
