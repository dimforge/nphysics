use std::env;
use std::rc::Rc;
use std::cell::RefCell;
use sfml::graphics::{RenderWindow, RenderTarget, Font};
use sfml::window::{ContextSettings, VideoMode, Close};
use sfml::window::event;
use sfml::window::keyboard::Key;
use sfml::window::mouse::MouseButton;
use sfml::graphics::Color;
use sfml::system::vector2::Vector2i;
use na::{Pnt2, Pnt3, Iso2};
use na;
use ncollide::world::CollisionGroups;
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics::detection::joint::{Fixed, Anchor};
use camera::Camera;
use fps::Fps;
use engine::GraphicsManager;
use draw_helper;

fn usage(exe_name: &str) {
    println!("Usage: {} [OPTION] ", exe_name);
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


#[derive(PartialEq)]
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

struct TestbedState<'a> {
    running: RunMode,
    draw_colls: bool,
    camera: Camera,
    fps: Fps<'a>,
    grabbed_object: Option<Rc<RefCell<RigidBody>>>,
    grabbed_object_joint: Option<Rc<RefCell<Fixed>>>,
}

impl<'a> TestbedState<'a> {
    fn new(fnt: &'a Font) -> TestbedState<'a> {
        TestbedState{
            running: RunMode::Running,
            draw_colls: false,
            camera: Camera::new(),
            fps: Fps::new(&fnt),
            grabbed_object: None,
            grabbed_object_joint: None,
        }
    }
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
                None          => panic!("Error on creating the sfml window.")
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
        let font_mem = include_bytes!("Inconsolata.otf");
        let     fnt  = Font::new_from_memory(font_mem).unwrap();

        let mut state = TestbedState::new(&fnt);

        let mut args    = env::args();

        if args.len() > 1 {
            let exname = args.next().unwrap();
            for arg in args {
                if &arg[..] == "--help" || &arg[..] == "-h" {
                    usage(&exname[..]);
                    return;
                }
                else if &arg[..] == "--pause" {
                    state.running = RunMode::Stop;
                }
            }
        }

        self.window.set_framerate_limit(60);

        self.run_loop(state);

        self.window.close();
    }

    fn run_loop(&mut self, mut state: TestbedState) {
        while self.window.is_open() {
            self.process_events(&mut state);

            self.window.clear(&Color::black());

            state.fps.reset();

            self.progress_world(&mut state);

            state.fps.register_delta();

            self.graphics.draw(&mut self.window, &state.camera);

            state.camera.activate_scene(&mut self.window);
            self.draw_collisions(&mut state);

            state.camera.activate_ui(&mut self.window);
            state.fps.draw_registered(&mut self.window);

            self.window.display();
        }
    }

    fn process_events(&mut self, mut state: &mut TestbedState) {
        loop {
            match self.window.poll_event() {
                event::KeyPressed{code, ..} => self.process_key_press(&mut state, code),
                event::MouseButtonPressed{button, x, y} => self.process_mouse_press(&mut state, button, x, y),
                event::MouseButtonReleased{button, x, y} => self.process_mouse_release(&mut state, button, x, y),
                event::MouseMoved{x, y} => self.process_mouse_moved(&mut state, x, y),
                event::Closed  => self.window.close(),
                event::NoEvent => break,
                e              => state.camera.handle_event(&e)
            }
        }
    }

    fn process_key_press(&mut self, state: &mut TestbedState, code: Key) {
        match code {
            Key::Escape => self.window.close(),
            Key::S      => state.running = RunMode::Step,
            Key::Space  => state.draw_colls = !state.draw_colls,
            Key::T      => {
                if state.running == RunMode::Stop {
                    state.running = RunMode::Running;
                }
                else {
                    state.running = RunMode::Stop;
                }
            },
            _                => { }
        }
    }

    fn process_mouse_press(&mut self, state: &mut TestbedState, button: MouseButton, x: i32, y: i32) {
        match button {
            MouseButton::MouseLeft => {
                let mapped_coords = state.camera.map_pixel_to_coords(Vector2i::new(x, y));
                let mapped_point = Pnt2::new(mapped_coords.x, mapped_coords.y);
                for b in self.world
                             .collision_world()
                             .interferences_with_point(&mapped_point, &CollisionGroups::new()) {
                    if b.data.borrow().can_move() {
                        state.grabbed_object = Some(b.data.clone())
                    }
                }

                match state.grabbed_object {
                    Some(ref b) => {
                        match state.grabbed_object_joint {
                            Some(ref j) => self.world.remove_fixed(j),
                            None        => { }
                        }

                        let _1: Iso2<f32> = na::one();
                        let attach2 = na::append_translation(&_1, mapped_point.as_vec());
                        let attach1 = na::inv(&na::transformation(b.borrow().position())).unwrap() * attach2;
                        let anchor1 = Anchor::new(Some(state.grabbed_object.as_ref().unwrap().clone()), attach1);
                        let anchor2 = Anchor::new(None, attach2);
                        let joint = Fixed::new(anchor1, anchor2);
                        state.grabbed_object_joint = Some(self.world.add_fixed(joint));

                        for node in self.graphics.body_to_scene_node(b).unwrap().iter_mut() {
                            node.select()
                        }
                    },
                    None => { }
                }
            },
            _ => {
                state.camera.handle_event(&event::MouseButtonPressed{ button: button, x: x, y: y })
            }
        }
    }

    fn process_mouse_release(&mut self, state: &mut TestbedState, button: MouseButton, x: i32, y: i32) {
        match button {
            MouseButton::MouseLeft => {
                match state.grabbed_object {
                    Some(ref b) => {
                        for node in self.graphics.body_to_scene_node(b).unwrap().iter_mut() {
                            node.unselect()
                        }
                    },
                    None => { }
                }

                match state.grabbed_object_joint {
                    Some(ref j) => self.world.remove_fixed(j),
                    None => { }
                }

                state.grabbed_object = None;
                state.grabbed_object_joint = None;
            },
            _ => {
                state.camera.handle_event(&event::MouseButtonReleased{ button: button, x: x, y: y })
            }
        }
    }

    fn process_mouse_moved(&mut self, state: &mut TestbedState, x: i32, y: i32) {
        let mapped_coords = state.camera.map_pixel_to_coords(Vector2i::new(x, y));
        let mapped_point = Pnt2::new(mapped_coords.x, mapped_coords.y);
        let _1: Iso2<f32> = na::one();
        let attach2 = na::append_translation(&_1, (mapped_point).as_vec());
        match state.grabbed_object {
            Some(_) => {
                let joint = state.grabbed_object_joint.as_ref().unwrap();
                joint.borrow_mut().set_local2(attach2);
            },
            None => state.camera.handle_event(&event::MouseMoved{x: x, y: y})
        };
    }

    fn progress_world(&mut self, state: &mut TestbedState) {
        if state.running != RunMode::Stop {
            self.world.step(0.016);
        }

        if state.running == RunMode::Step {
            state.running = RunMode::Stop;
        }
    }

    fn draw_collisions(&mut self, state: &mut TestbedState) {
        if state.draw_colls {
            draw_helper::draw_colls(&mut self.window, &mut self.world);
        }
    }
}
