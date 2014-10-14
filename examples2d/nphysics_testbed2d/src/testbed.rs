use std::rc::Rc;
use std::cell::RefCell;
use std::os;
use std::num::{Zero, One};
use rsfml::graphics::{RenderWindow, RenderTarget, Font};
use rsfml::window::{ContextSettings, VideoMode, Close};
use rsfml::window::event;
use rsfml::window::{keyboard, mouse};
use rsfml::graphics::Color;
use rsfml::system::vector2::Vector2i;
use na::{Pnt2, Pnt3, Vec2, Iso2};
use na;
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics::detection::joint::{Fixed, Anchor};
use camera::Camera;
use fps::Fps;
use engine::GraphicsManager;
use draw_helper;
use ncollide::ray::Ray;

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

    pub fn set_color(&mut self, body: &Rc<RefCell<RigidBody>>, color: Pnt3<f32>) {
        let color = Pnt3::new(
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
        let mut grabbed_object: Option<Rc<RefCell<RigidBody>>> = None;
        let mut grabbed_object_joint: Option<Rc<RefCell<Fixed>>> = None;

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
                    event::MouseButtonPressed{button, x, y} => {
                        match button {
                            mouse::MouseLeft => {
                                let mapped_coords = camera.map_pixel_to_coords(Vector2i::new(x, y));
                                let mapped_point = Pnt2::new(mapped_coords.x, mapped_coords.y);
                                let ray = Ray::new(mapped_point, Vec2::new(1.0, 0.0));
                                let mut interferences = Vec::new();
                                self.world.cast_ray(&ray, &mut interferences);

                                let mut minb = None;

                                for (b, toi) in interferences.into_iter() {
                                    if toi.is_zero() {
                                        minb = Some(b);
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
                                        for node in self.graphics.body_to_scene_node(b).unwrap().iter_mut() {
                                            match grabbed_object_joint {
                                                Some(ref j) => self.world.remove_fixed(j),
                                                None        => { }
                                            }

                                            let _1: Iso2<f32> = One::one();
                                            let attach2 = na::append_translation(&_1, (ray.orig).as_vec());
                                            let attach1 = na::inv(&na::transformation(b.borrow().transform_ref())).unwrap() * attach2;
                                            let anchor1 = Anchor::new(Some(minb.as_ref().unwrap().clone()), attach1);
                                            let anchor2 = Anchor::new(None, attach2);
                                            let joint = Fixed::new(anchor1, anchor2);
                                            grabbed_object_joint = Some(self.world.add_fixed(joint));
                                            node.select()
                                        }
                                    },
                                    None => { }
                                }
                            },
                            _ => {
                                camera.handle_event(&event::MouseButtonPressed{ button: button, x: x, y: y })
                            }
                        }
                    },
                    event::MouseButtonReleased{button, x, y} => {
                        match button {
                            mouse::MouseLeft => {
                                match grabbed_object {
                                    Some(ref b) => {
                                        for node in self.graphics.body_to_scene_node(b).unwrap().iter_mut() {
                                            node.unselect()
                                        }
                                    },
                                    None => { }
                                }

                                match grabbed_object_joint {
                                    Some(ref j) => self.world.remove_fixed(j),
                                    None => { }
                                }

                                grabbed_object = None;
                                grabbed_object_joint = None;
                            },
                            _ => {
                                camera.handle_event(&event::MouseButtonReleased{ button: button, x: x, y: y })
                            }
                        }
                    }
                    event::MouseMoved{x, y} => {
                        let mapped_coords = camera.map_pixel_to_coords(Vector2i::new(x, y));
                        let mapped_point = Pnt2::new(mapped_coords.x, mapped_coords.y);
                        let _1: Iso2<f32> = One::one();
                        let attach2 = na::append_translation(&_1, (mapped_point).as_vec());
                        match grabbed_object {
                            Some(_) => {
                                let joint = grabbed_object_joint.as_ref().unwrap();
                                joint.borrow_mut().set_local2(attach2);
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
