use std::env;
use std::rc::Rc;
use std::cell::RefCell;
use sfml::graphics::{Font, RenderTarget, RenderWindow};
use sfml::window::{ContextSettings, VideoMode};
use sfml::window::window_style;
use sfml::window::event::Event;
use sfml::window::{Key, MouseButton};
use sfml::graphics::Color;
use sfml::system::Vector2i;
use na::{Isometry2, Point2, Point3};
use ncollide::world::CollisionGroups;
use nphysics2d::world::World;
use nphysics2d::object::BodyHandle;
use nphysics2d::joint::{ConstraintHandle, FixedConstraint};
use camera::Camera;
use fps::Fps;
use engine::{GraphicsManager, GraphicsManagerHandle};
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
    Step,
}

pub struct Testbed {
    world: World<f32>,
    callbacks: Vec<Box<Fn(&mut World<f32>, f32)>>,
    window: RenderWindow,
    graphics: GraphicsManagerHandle,
    time: f32,
}

struct TestbedState<'a> {
    running: RunMode,
    draw_colls: bool,
    camera: Camera,
    fps: Fps<'a>,
    grabbed_object: Option<BodyHandle>,
    grabbed_object_joint: Option<ConstraintHandle>,
}

impl<'a> TestbedState<'a> {
    fn new(fnt: &'a Font, rw: &RenderWindow) -> TestbedState<'a> {
        TestbedState {
            running: RunMode::Running,
            draw_colls: false,
            camera: Camera::new(rw),
            fps: Fps::new(&fnt),
            grabbed_object: None,
            grabbed_object_joint: None,
        }
    }
}

impl Testbed {
    pub fn new_empty() -> Testbed {
        let mode = VideoMode::new_init(800, 600, 32);
        let window = match RenderWindow::new(
            mode,
            "nphysics 2d demo",
            window_style::CLOSE,
            &ContextSettings::default(),
        ) {
            Some(rwindow) => rwindow,
            None => panic!("Error on creating the sfml window."),
        };
        Testbed {
            world: World::new(),
            callbacks: Vec::new(),
            window: window,
            graphics: Rc::new(RefCell::new(GraphicsManager::new())),
            time: 0.0,
        }
    }

    pub fn graphics(&self) -> GraphicsManagerHandle {
        self.graphics.clone()
    }

    pub fn new(world: World<f32>) -> Testbed {
        let mut res = Testbed::new_empty();

        res.set_world(world);

        res
    }

    pub fn set_world(&mut self, world: World<f32>) {
        let mut bgraphics = self.graphics.borrow_mut();

        self.world = world;
        bgraphics.clear();

        for co in self.world.colliders() {
            bgraphics.add(co.handle(), &self.world);
        }

        // for s in self.world.sensors() {
        //     bgraphics.add(WorldObject::Sensor(s.clone()));
        // }
    }

    pub fn set_body_color(&mut self, world: &World<f32>, body: BodyHandle, color: Point3<f32>) {
        self.graphics
            .borrow_mut()
            .set_body_color(world, body, color);
    }

    // pub fn set_sensor_color(&mut self, sensor: &SensorHandle<f32>, color: Point3<f32>) {
    //     self.graphics.borrow_mut().set_sensor_color(sensor, color);
    // }

    pub fn add_callback<F: Fn(&mut World<f32>, f32) + 'static>(&mut self, callback: F) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn run(&mut self) {
        let font_mem = include_bytes!("Inconsolata.otf");
        let fnt = Font::new_from_memory(font_mem).unwrap();

        let mut state = TestbedState::new(&fnt, &self.window);

        let mut args = env::args();

        if args.len() > 1 {
            let exname = args.next().unwrap();
            for arg in args {
                if &arg[..] == "--help" || &arg[..] == "-h" {
                    usage(&exname[..]);
                    return;
                } else if &arg[..] == "--pause" {
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

            self.window.clear(&Color::new_rgb(250, 250, 250));

            state.fps.reset();

            self.progress_world(&mut state);

            state.fps.register_delta();

            self.graphics
                .borrow_mut()
                .draw(&mut self.window, &state.camera, &self.world);

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
                Event::KeyPressed { code, .. } => self.process_key_press(&mut state, code),
                Event::MouseButtonPressed { button, x, y } => {
                    self.process_mouse_press(&mut state, button, x, y)
                }
                Event::MouseButtonReleased { button, x, y } => {
                    self.process_mouse_release(&mut state, button, x, y)
                }
                Event::MouseMoved { x, y } => self.process_mouse_moved(&mut state, x, y),
                Event::Closed => self.window.close(),
                Event::NoEvent => break,
                e => state.camera.handle_event(&e),
            }
        }
    }

    fn process_key_press(&mut self, state: &mut TestbedState, code: Key) {
        match code {
            Key::Escape => self.window.close(),
            Key::S => state.running = RunMode::Step,
            Key::Space => state.draw_colls = !state.draw_colls,
            Key::T => {
                if state.running == RunMode::Stop {
                    state.running = RunMode::Running;
                } else {
                    state.running = RunMode::Stop;
                }
            }
            _ => {}
        }
    }

    fn process_mouse_press(
        &mut self,
        state: &mut TestbedState,
        button: MouseButton,
        x: i32,
        y: i32,
    ) {
        match button {
            MouseButton::Left => {
                let mapped_coords = state.camera.map_pixel_to_coords(Vector2i::new(x, y));
                let mapped_point = Point2::new(mapped_coords.x, mapped_coords.y);
                // FIXME: use the collision groups to filter out sensors.
                let all_groups = &CollisionGroups::new();
                for b in self.world
                    .collision_world()
                    .interferences_with_point(&mapped_point, all_groups)
                {
                    if !b.query_type().is_proximity_query() && !b.data().body().is_ground() {
                        state.grabbed_object = Some(b.data().body())
                    }
                }

                if let Some(body) = state.grabbed_object {
                    if let Some(joint) = state.grabbed_object_joint {
                        let _ = self.world.remove_constraint(joint);
                    }

                    let body_pos = self.world.body_part(body).position();
                    let attach1 = Isometry2::new(mapped_point.coords, 0.0);
                    let attach2 = body_pos.inverse() * attach1;
                    let joint = FixedConstraint::new(BodyHandle::ground(), body, attach1, attach2);
                    state.grabbed_object_joint = Some(self.world.add_constraint(joint));

                    for node in self.graphics
                        .borrow_mut()
                        .rigid_body_to_scene_node(body)
                        .unwrap()
                        .iter_mut()
                    {
                        node.select()
                    }
                }
            }
            _ => state.camera.handle_event(&Event::MouseButtonPressed {
                button: button,
                x: x,
                y: y,
            }),
        }
    }

    fn process_mouse_release(
        &mut self,
        state: &mut TestbedState,
        button: MouseButton,
        x: i32,
        y: i32,
    ) {
        match button {
            MouseButton::Left => {
                if let Some(body) = state.grabbed_object {
                    for node in self.graphics
                        .borrow_mut()
                        .rigid_body_to_scene_node(body)
                        .unwrap()
                        .iter_mut()
                    {
                        node.unselect()
                    }
                }

                if let Some(joint) = state.grabbed_object_joint {
                    let _ = self.world.remove_constraint(joint);
                }

                state.grabbed_object = None;
                state.grabbed_object_joint = None;
            }
            _ => state.camera.handle_event(&Event::MouseButtonReleased {
                button: button,
                x: x,
                y: y,
            }),
        }
    }

    fn process_mouse_moved(&mut self, state: &mut TestbedState, x: i32, y: i32) {
        let mapped_coords = state.camera.map_pixel_to_coords(Vector2i::new(x, y));
        let mapped_point = Point2::new(mapped_coords.x, mapped_coords.y);
        let attach2 = Isometry2::new(mapped_point.coords, 0.0);
        match state.grabbed_object {
            Some(_) => {
                let joint = state.grabbed_object_joint.unwrap();
                let joint = self.world
                    .constraint_mut(joint)
                    .downcast_mut::<FixedConstraint<f32>>()
                    .unwrap();
                joint.set_anchor_1(attach2);
            }
            None => state.camera.handle_event(&Event::MouseMoved { x: x, y: y }),
        };
    }

    fn progress_world(&mut self, state: &mut TestbedState) {
        if state.running != RunMode::Stop {
            for f in &self.callbacks {
                f(&mut self.world, self.time)
            }
            self.world.step();
            self.time += self.world.timestep();
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
