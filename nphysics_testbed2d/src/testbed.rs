use crate::engine::GraphicsManager;
use kiss3d::camera::Camera;
use kiss3d::event::{Action, Key, Modifiers, WindowEvent};
use kiss3d::loader::obj;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use na::{self, Point2, Point3};
use ncollide2d::utils::GenerationalId;
use ncollide2d::query::Ray;
use ncollide2d::world::CollisionGroups;
use nphysics2d::joint::{ConstraintHandle, MouseConstraint};
use nphysics2d::object::{BodyHandle, BodyPartHandle, ColliderHandle, ColliderAnchor};
use nphysics2d::world::World;
use std::collections::HashMap;
use std::env;
use std::mem;
use std::path::Path;
use std::rc::Rc;

#[derive(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step,
}

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
    println!(
        "    CTRL + left click + drag - select and drag an object using a ball-in-socket joint."
    );
    println!("    SHIFT + left click - remove an object.");
    println!("    arrows - move around when in first-person camera mode.");
    println!("    space  - switch wireframe mode. When ON, the contacts points and normals are displayed.");
    println!("    b      - draw the bounding boxes.");
}

pub struct Testbed {
    world: World<f32>,
    window: Option<Box<Window>>,
    graphics: GraphicsManager,
    nsteps: usize,
    callbacks: Vec<Box<Fn(&mut World<f32>, &mut GraphicsManager, f32)>>,
    time: f32,
    hide_counters: bool,
    persistant_contacts: HashMap<GenerationalId, bool>,

    font: Rc<Font>,
    running: RunMode,
    draw_colls: bool,
    cursor_pos: Point2<f32>,
    grabbed_object: Option<BodyPartHandle>,
    grabbed_object_constraint: Option<ConstraintHandle>,
    drawing_ray: Option<Point2<f32>>
}

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let world = World::new();

        let mut window = Box::new(Window::new("nphysics: 2d demo"));
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_framerate_limit(Some(60));

        Testbed {
            world: world,
            callbacks: Vec::new(),
            window: Some(window),
            graphics: graphics,
            nsteps: 1,
            time: 0.0,
            hide_counters: false,
            persistant_contacts: HashMap::new(),

            font: Font::default(),
            running: RunMode::Running,
            draw_colls: false,
            cursor_pos: Point2::new(0.0f32, 0.0),
            grabbed_object: None,
            grabbed_object_constraint: None,
            drawing_ray: None
        }
    }

    pub fn new(world: World<f32>) -> Testbed {
        let mut res = Testbed::new_empty();

        res.set_world(world);

        res
    }

    pub fn set_number_of_steps_per_frame(&mut self, nsteps: usize) {
        self.nsteps = nsteps
    }

    pub fn hide_performance_counters(&mut self) {
        self.hide_counters = true;
    }

    pub fn show_performance_counters(&mut self) {
        self.hide_counters = false;
    }

    pub fn set_world(&mut self, world: World<f32>) {
        self.world = world;
        self.world.enable_performance_counters();

        self.graphics.clear(self.window.as_mut().unwrap());

        for co in self.world.colliders() {
            self.graphics
                .add(self.window.as_mut().unwrap(), co.handle(), &self.world);
        }
    }

    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        self.graphics.look_at(at, zoom);
    }

    pub fn set_body_color(&mut self, body: BodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(body, color);
    }

    pub fn set_collider_color(&mut self, collider: ColliderHandle, color: Point3<f32>) {
        self.graphics.set_collider_color(collider, color);
    }

    pub fn world(&self) -> &World<f32> {
        &self.world
    }

    pub fn graphics_mut(&mut self) -> &mut GraphicsManager {
        &mut self.graphics
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path = Path::new(path);
        let empty = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "")
            .ok()
            .expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().unwrap().to_owned().unwrap();
            let indices = m.faces().read().unwrap().to_owned().unwrap();

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

    pub fn add_callback<F: Fn(&mut World<f32>, &mut GraphicsManager, f32) + 'static>(
        &mut self,
        callback: F,
    ) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn run(mut self) {
        let mut args = env::args();

        if args.len() > 1 {
            let exname = args.next().unwrap();
            for arg in args {
                if &arg[..] == "--help" || &arg[..] == "-h" {
                    usage(&exname[..]);
                    return;
                } else if &arg[..] == "--pause" {
                    self.running = RunMode::Stop;
                }
            }
        }

        let window = mem::replace(&mut self.window, None).unwrap();
        window.render_loop(self);
    }
}

impl State for Testbed {
    fn cameras_and_effect(
        &mut self,
    ) -> (
        Option<&mut Camera>,
        Option<&mut PlanarCamera>,
        Option<&mut PostProcessingEffect>,
    ) {
        (None, Some(self.graphics.camera_mut()), None)
    }

    fn step(&mut self, window: &mut Window) {
        for mut event in window.events().iter() {
            match event.value {
                //         WindowEvent::MouseButton(MouseButton::Button2, Action::Press, Key::LControl) |
                //         WindowEvent::MouseButton(MouseButton::Button2, Action::Press, Key::RControl)  => {
                //             let mut graphics = self.graphics;
                //             let geom   = Cuboid::new(Vector3::new(0.5f32, 0.5f32, 0.5f32));
                //             let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                //             let size = window.size();
                //             let (pos, dir) = graphics.camera().unproject(&cursor_pos, &size);

                //             rb.set_translation(Translation3::from_vector(pos.coords));
                //             rb.set_lin_vel(dir * 1000.0f32);

                //             let body = self.world.add_rigid_body(rb);
                //             self.world.add_ccd_to(&body, 1.0, false);
                //             graphics.add(window, WorldObject::RigidBody(body));
                //         },
                WindowEvent::MouseButton(_, Action::Press, modifier) => {
                    let all_groups = &CollisionGroups::new();
                    for b in self
                        .world
                        .collider_world()
                        .interferences_with_point(&self.cursor_pos, all_groups)
                        {
                            if !b.query_type().is_proximity_query() && !b.body().is_ground() {

                                if
                                    let ColliderAnchor::OnBodyPart { body_part, .. } = b.anchor()
                                    {
                                        self.grabbed_object = Some(*body_part);
                                    } else { continue; }
                            }
                        }

                    if modifier.contains(Modifiers::Shift) {
                        if let Some(body_part) = self.grabbed_object {
                            if !body_part.is_ground() {
                                self.graphics.remove_body_nodes(window, body_part.0);
                                self.world.remove_bodies(&[body_part.0]);
                            }
                        }

                        self.grabbed_object = None;
                    } else if modifier.contains(Modifiers::Control) {
                        if let Some(body) = self.grabbed_object {
                            if let Some(joint) = self.grabbed_object_constraint {
                                let _ = self.world.remove_constraint(joint);
                            }

                            let body_pos = self.world.body(body.0).unwrap().part(body.1).unwrap().position();
                            let attach1 = self.cursor_pos;
                            let attach2 = body_pos.inverse() * attach1;
                            let joint = MouseConstraint::new(
                                BodyPartHandle::ground(),
                                body,
                                attach1,
                                attach2,
                                1.0,
                            );
                            self.grabbed_object_constraint = Some(self.world.add_constraint(joint));

                            for node in self
                                .graphics
                                .body_nodes_mut(body.0)
                                .unwrap()
                                .iter_mut()
                                {
                                    node.select()
                                }
                        }

                        event.inhibited = true;
                    } else if modifier.contains(Modifiers::Alt) {
                        self.drawing_ray = Some(self.cursor_pos);
                    } else {
                        self.grabbed_object = None;
                    }
                }
                WindowEvent::MouseButton(_, Action::Release, _) => {
                    if let Some(body) = self.grabbed_object {
                        for n in self
                            .graphics
                            .body_nodes_mut(body.0)
                            .unwrap()
                            .iter_mut()
                            {
                                n.unselect()
                            }
                    }

                    if let Some(joint) = self.grabbed_object_constraint {
                        let _ = self.world.remove_constraint(joint);
                    }


                    if let Some(start) = self.drawing_ray {
                        self.graphics.add_ray(Ray::new(start, self.cursor_pos - start));
                    }

                    self.drawing_ray = None;
                    self.grabbed_object = None;
                    self.grabbed_object_constraint = None;
                }
                WindowEvent::CursorPos(x, y, modifiers) => {
                    self.cursor_pos.x = x as f32;
                    self.cursor_pos.y = y as f32;

                    self.cursor_pos = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(window.size()));

                    let attach2 = self.cursor_pos;
                    if let Some(_) = self.grabbed_object {
                        let joint = self.grabbed_object_constraint.unwrap();
                        let joint = self
                            .world
                            .constraint_mut(joint)
                            .downcast_mut::<MouseConstraint<f32>>()
                            .unwrap();
                        joint.set_anchor_1(attach2);
                    }

                    event.inhibited = modifiers.contains(Modifiers::Control)
                        || modifiers.contains(Modifiers::Shift);
                }
                WindowEvent::Key(Key::T, Action::Release, _) => {
                    if self.running == RunMode::Stop {
                        self.running = RunMode::Running;
                    } else {
                        self.running = RunMode::Stop;
                    }
                }
                WindowEvent::Key(Key::S, Action::Release, _) => self.running = RunMode::Step,
                //         WindowEvent::Key(Key::B, _, Action::Release, _) => {
                //             // XXX: there is a bug on kiss3d with the removal of objects.
                //             // draw_aabbs = !draw_aabbs;
                //             // if draw_aabbs {
                //             //     graphics.enable_aabb_draw(window);
                //             // }
                //             // else {
                //             //     graphics.disable_aabb_draw(window);
                //             // }
                //         },
                WindowEvent::Key(Key::Space, Action::Release, _) => {
                    self.draw_colls = !self.draw_colls;
                    for co in self.world.colliders() {
                        // FIXME: ugly clone.
                        if let Some(ns) =
                        self.graphics.body_nodes_mut(co.body())
                            {
                                for n in ns.iter_mut() {
                                    if let Some(node) = n.scene_node_mut() {
                                        if self.draw_colls {
                                            node.set_lines_width(1.0);
                                            node.set_surface_rendering_activation(false);
                                        } else {
                                            node.set_lines_width(0.0);
                                            node.set_surface_rendering_activation(true);
                                        }
                                    }
                                }
                            }
                    }
                }
                //      WindowEvent::Key(Key::Num1, _, Action::Press, _) => {
                //          let mut graphics = self.graphics;
                //          let geom   = Ball::new(0.5f32);
                //          let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                //          let cam_transfom;

                //          {
                //              let cam      = graphics.camera();
                //              cam_transfom = cam.view_transform().inverse()
                //          }

                //          rb.append_translation(&cam_transfom.translation);

                //          let front = cam_transfom.rotation * -Vector3::z();

                //          rb.set_lin_vel(front * 40.0f32);

                //          let body = self.world.add_rigid_body(rb);
                //          graphics.add(window, body, &self.world.rigid_bodies());
                //      },
                //      WindowEvent::Key(Key::Num2, _, Action::Press, _) => {
                //          let mut graphics = self.graphics;
                //          let geom   = Cuboid::new(Vector3::new(0.5f32, 0.5, 0.5));
                //          let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                //          let cam_transform;

                //          {
                //              let cam = graphics.camera();
                //              cam_transform = cam.view_transform().inverse()
                //          }

                //          rb.append_translation(&cam_transform.translation);

                //          let front = cam_transform.rotation * -Vector3::z();

                //          rb.set_lin_vel(front * 40.0f32);

                //          let body = self.world.add_rigid_body(rb);
                //          graphics.add(window, body, &self.world.rigid_bodies());
                //      }
                _ => {}
            }
        }

        if self.running != RunMode::Stop {
            for _ in 0..self.nsteps {
                for f in &self.callbacks {
                    f(&mut self.world, &mut self.graphics, self.time)
                }
                self.world.step();
                if !self.hide_counters {
                    println!("{}", self.world.performance_counters());
                }
                self.time += self.world.timestep();
            }
        }

        self.graphics.draw(&self.world, window);

        if self.draw_colls {
            draw_collisions(
                window,
                &mut self.world,
                &mut self.persistant_contacts,
                self.running != RunMode::Stop,
            );
        }

        if self.running == RunMode::Step {
            self.running = RunMode::Stop;
        }

        if let Some(start) = self.drawing_ray {
            window.draw_planar_line(&start, &self.cursor_pos, &Point3::new(1.0, 0.0, 0.0));
        }

        let color = Point3::new(0.0, 0.0, 0.0);

        if true {
            //running != RunMode::Stop {
            window.draw_text(
                &format!(
                    "Simulation time: {:.*}sec.",
                    4,
                    self.world.performance_counters().step_time(),
                )[..],
                &Point2::origin(),
                60.0,
                &self.font,
                &color,
            );
        } else {
            window.draw_text("Paused", &Point2::origin(), 60.0, &self.font, &color);
        }
        window.draw_text(CONTROLS, &Point2::new(0.0, 75.0), 40.0, &self.font, &color);
    }
}

const CONTROLS: &'static str = "Controls:
    Ctrl + click + drag: select and move a solid.
    Right click + drag: pan the camera.
    Mouse wheel: zoom in/zoom out.
    T: pause/resume simulation.
    S: step simulation.";

fn draw_collisions(
    window: &mut Window,
    world: &World<f32>,
    existing: &mut HashMap<GenerationalId, bool>,
    running: bool,
) {
    for (_, _, _, manifold) in world.collider_world().contact_pairs() {
        for c in manifold.contacts() {
            if existing.contains_key(&c.id) {
                if running {
                    existing.insert(c.id, true);
                }
            } else {
                existing.insert(c.id, false);
            }

            let color = if c.contact.depth < 0.0 {// existing[&c.id] {
                Point3::new(0.0, 0.0, 1.0)
            } else {
                Point3::new(1.0, 0.0, 0.0)
            };

            window.draw_planar_line(&c.contact.world1, &c.contact.world2, &color);

//            let center = na::center(&c.contact.world1, &c.contact.world2);
//            let end = center + *c.contact.normal * 0.4f32;
//            window.draw_planar_line(&center, &end, &Point3::new(0.0, 1.0, 1.0))
        }
    }
}
