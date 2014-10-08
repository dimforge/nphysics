use std::rc::Rc;
use std::cell::RefCell;
use std::os;
use rsfml::graphics::{RenderWindow, RenderTarget, Font};
use rsfml::window::{ContextSettings, VideoMode, Close};
use rsfml::window::event;
use rsfml::window::keyboard;
use rsfml::graphics::Color;
use na::{Vec2, Vec3};
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
}


#[deriving(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step
}

pub struct Testbed<'a> {
    world:    World,
    window:   RenderWindow,
    graphics: GraphicsManager<'a>
}

impl<'a> Testbed<'a> {
    pub fn new_empty() -> Testbed<'a> {
        let mode    = VideoMode::new_init(800, 600, 32);
        let setting = ContextSettings {
            depth_bits:         10,
            stencil_bits:       10,
            antialiasing_level: 2,
            major_version:      0,
            minor_version:      1
        };
        let window =
            match RenderWindow::new(mode, "nphysics 2d demo", Close, &setting) {
                Some(rwindow) => rwindow,
                None          => fail!("Error on creating the sfml window.")
            };
        let graphics = GraphicsManager::new();

        Testbed {
            world:    World::new(),
            window:   window,
            graphics: graphics
        }
    }

    pub fn new(world: World) -> Testbed<'a> {
        let mut res = Testbed::new_empty();

        res.set_world(world);

        res
    }

    pub fn set_world(&mut self, world: World) {
        self.world = world;
        self.graphics.clear();

        for rb in self.world.bodies() {
            self.graphics.add(rb.clone());
        }
    }

    pub fn set_color(&mut self, body: &Rc<RefCell<RigidBody>>, color: Vec3<f32>) {
        let color = Vec3::new(
            (color.x * 255.0) as u8,
            (color.y * 255.0) as u8,
            (color.z * 255.0) as u8
        );

        self.graphics.set_color(body, color);
    }

    pub fn run(&mut self) {
        let args        = os::args();
        let mut running = Running;

        if args.len() > 1 {
            if args.len() > 2 || args[1].as_slice() != "--pause" {
                usage(args[0].as_slice());
                os::set_exit_status(1);
                return;
            }
            else {
                running = Stop;
            }
        }

        let mut draw_colls = false;


        let mut camera = Camera::new();

        self.window.set_framerate_limit(60);


        let font_mem = include_bin!("Inconsolata.otf");
        let     fnt  = Font::new_from_memory(font_mem).unwrap();
        let mut fps  = Fps::new(&fnt);
        let mut cursor_pos;
        let grabbed_object: Option<Rc<RefCell<RigidBody>>> = None;
        let grabbed_object_joint: Option<Rc<RefCell<BallInSocket>>> = None;

        while self.window.is_open() {
            loop {
                match self.window.poll_event() {
                    event::KeyPressed{code, ..} => {
                        match code {
                            keyboard::Escape => self.window.close(),
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
                    event::Closed  => self.window.close(),
                    event::NoEvent => break,
                    e              => camera.handle_event(&e)
                }
            }

            self.window.clear(&Color::black());

            fps.reset();

            if running != Stop {
                self.world.step(0.016);
            }

            if running == Step {
                running = Stop;
            }
            fps.register_delta();
            self.graphics.draw(&mut self.window, &camera);

            camera.activate_scene(&mut self.window);
            if draw_colls {
                draw_helper::draw_colls(&mut self.window, &mut self.world);
            }

            camera.activate_ui(&mut self.window);
            fps.draw_registered(&mut self.window);

            self.window.display();
        }

        self.window.close();
    }
}
