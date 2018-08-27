use engine::GraphicsManager;
use kiss3d::camera::Camera;
use kiss3d::event::{Action, Key, Modifiers, WindowEvent};
use kiss3d::loader::obj;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use na::{self, Point2, Point3};
use ncollide2d::utils::GenerationalId;
use ncollide2d::world::CollisionGroups;
use nphysics2d::joint::{ConstraintHandle, MouseConstraint};
use nphysics2d::object::{BodyHandle, ColliderHandle};
use nphysics2d::world::World;
use std::collections::HashMap;
use std::env;
use std::mem;
use std::path::Path;
use std::rc::Rc;
use world_owner::{WorldOwner, WorldOwnerExclusive, WorldOwnerShared};

#[derive(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step,
}

fn usage(exe_name: &str) {
    info!("Usage: {} [OPTION] ", exe_name);
    info!("");
    info!("Options:");
    info!("    --help  - prints this help message and exits.");
    info!("    --pause - do not start the simulation right away.");
    info!("");
    info!("The following keyboard commands are supported:");
    info!("    t      - pause/continue the simulation.");
    info!("    s      - pause then execute only one simulation step.");
    info!("    1      - launch a ball.");
    info!("    2      - launch a cube.");
    info!("    3      - launch a fast cube using continuous collision detection.");
    info!("    TAB    - switch camera mode (first-person or arc-ball).");
    info!("    SHIFT + right click - launch a fast cube using continuous collision detection.");
    info!(
        "    CTRL + left click + drag - select and drag an object using a ball-in-socket joint."
    );
    info!("    SHIFT + left click - remove an object.");
    info!("    arrows - move around when in first-person camera mode.");
    info!("    space  - switch wireframe mode. When ON, the contacts points and normals are displayed.");
    info!("    b      - draw the bounding boxes.");
}

pub struct Testbed {
    window: Option<Box<Window>>,
    graphics: GraphicsManager,
    nsteps: usize,
    callbacks: Vec<Box<Fn(&mut Box<WorldOwner>, &mut GraphicsManager, f32)>>,
    time: f32,
    hide_counters: bool,
    persistant_contacts: HashMap<GenerationalId, bool>,

    font: Rc<Font>,
    running: RunMode,
    draw_colls: bool,
    cursor_pos: Point2<f32>,
    grabbed_object: Option<BodyHandle>,
    grabbed_object_constraint: Option<ConstraintHandle>,
    world: Box<WorldOwner>,
}

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let world = World::new();

        let mut window = Box::new(Window::new("nphysics: 2d demo"));
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_framerate_limit(Some(60));

        Testbed {
            world: Box::new(WorldOwnerShared::new(world)),
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
        }
    }

    pub fn new(world: World<f32>) -> Testbed {
        Testbed::new_with_world_owner(Box::new(WorldOwnerExclusive::from(world)))
    }
    pub fn new_with_world_owner(world_owner: Box<WorldOwner>) -> Testbed {
        let mut res = Testbed::new_empty();

        res.set_world_owner(world_owner);

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
        self.set_world_owner(Box::new(WorldOwnerExclusive::from(world)));
    }

    pub fn set_world_owner(&mut self, world: Box<WorldOwner>) {
        self.world = world;
        let mut world = self.world.get_mut();
        world.enable_performance_counters();

        self.graphics.clear(self.window.as_mut().unwrap());

        for co in world.colliders() {
            self.graphics
                .add(self.window.as_mut().unwrap(), co.handle(), &world);
        }
    }

    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        self.graphics.look_at(at, zoom);
    }

    pub fn set_body_color(&mut self, world: &World<f32>, body: BodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(world, body, color);
    }

    pub fn set_collider_color(&mut self, collider: ColliderHandle, color: Point3<f32>) {
        self.graphics.set_collider_color(collider, color);
    }

    pub fn world(&self) -> &Box<WorldOwner> {
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

    pub fn add_callback<F: Fn(&mut Box<WorldOwner>, &mut GraphicsManager, f32) + 'static>(
        &mut self,
        callback: F,
    ) where
        for<'r, 's> F: Fn(&'r mut Box<WorldOwner + 'static>, &'s mut GraphicsManager, f32),
    {
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
                    let mut physics_world = &mut self.world.get_mut();
                    let mapped_point = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(window.size()));

                    let all_groups = &CollisionGroups::new();
                    for b in physics_world
                        .collision_world()
                        .interferences_with_point(&mapped_point, all_groups)
                    {
                        if !b.query_type().is_proximity_query() && !b.data().body().is_ground() {
                            self.grabbed_object = Some(b.data().body());
                        }
                    }

                    if modifier.contains(Modifiers::Shift) {
                        if let Some(body) = self.grabbed_object {
                            if !body.is_ground() {
                                if !modifier.contains(Modifiers::Control) {
                                    self.graphics
                                        .remove_body_nodes(&physics_world, window, body);
                                    physics_world.remove_bodies(&[body]);
                                } else {
                                    if physics_world.multibody_link(body).is_some() {
                                        let key = self.graphics.remove_body_part_nodes(
                                            &physics_world,
                                            window,
                                            body,
                                        );
                                        physics_world.remove_multibody_links(&[body]);
                                        // FIXME: this is a bit ugly.
                                        self.graphics
                                            .update_after_body_key_change(&physics_world, key);
                                    }
                                }
                            }
                        }

                        self.grabbed_object = None;
                    } else if modifier.contains(Modifiers::Control) {
                        if let Some(body) = self.grabbed_object {
                            if let Some(joint) = self.grabbed_object_constraint {
                                let _ = physics_world.remove_constraint(joint);
                            }

                            let body_pos = physics_world.body_part(body).position();
                            let attach1 = mapped_point;
                            let attach2 = body_pos.inverse() * attach1;
                            let joint = MouseConstraint::new(
                                BodyHandle::ground(),
                                body,
                                attach1,
                                attach2,
                                1.0,
                            );
                            self.grabbed_object_constraint =
                                Some(physics_world.add_constraint(joint));

                            for node in self
                                .graphics
                                .body_nodes_mut(physics_world, body)
                                .unwrap()
                                .iter_mut()
                            {
                                node.select()
                            }
                        }

                        event.inhibited = true;
                    } else {
                        self.grabbed_object = None;
                    }
                }
                WindowEvent::MouseButton(_, Action::Release, _) => {
                    let mut physics_world = &mut self.world.get_mut();
                    if let Some(body) = self.grabbed_object {
                        for n in self
                            .graphics
                            .body_nodes_mut(physics_world, body)
                            .unwrap()
                            .iter_mut()
                        {
                            n.unselect()
                        }
                    }

                    if let Some(joint) = self.grabbed_object_constraint {
                        let _ = physics_world.remove_constraint(joint);
                    }

                    self.grabbed_object = None;
                    self.grabbed_object_constraint = None;
                }
                WindowEvent::CursorPos(x, y, modifiers) => {
                    let mut physics_world = &mut self.world.get_mut();
                    self.cursor_pos.x = x as f32;
                    self.cursor_pos.y = y as f32;

                    let mapped_point = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(window.size()));

                    let attach2 = mapped_point;
                    if let Some(_) = self.grabbed_object {
                        let joint = self.grabbed_object_constraint.unwrap();
                        let joint = physics_world
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
                    let mut physics_world = &mut self.world.get_mut();
                    self.draw_colls = !self.draw_colls;
                    for co in physics_world.colliders() {
                        // FIXME: ugly clone.
                        if let Some(ns) = self
                            .graphics
                            .body_nodes_mut(&physics_world, co.data().body())
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
                self.world.get_mut().step();
                if !self.hide_counters {
                    debug!("{}", self.world.get().performance_counters());
                }
                self.time += self.world.get().timestep();
            }
            let physics_world = &self.world.get();

            for co in physics_world.colliders() {
                if self
                    .graphics
                    .body_nodes_mut(physics_world, co.data().body())
                    .is_none()
                {
                    self.graphics.add(window, co.handle(), &physics_world);
                }
            }
            self.graphics.draw(&physics_world, window);
        }

        if self.draw_colls {
            draw_collisions(
                window,
                &mut self.world.get(),
                &mut self.persistant_contacts,
                self.running != RunMode::Stop,
            );
        }

        if self.running == RunMode::Step {
            self.running = RunMode::Stop;
        }

        let color = Point3::new(0.0, 0.0, 0.0);

        if true {
            //running != RunMode::Stop {
            window.draw_text(
                &format!(
                    "Simulation time: {:.*}sec.",
                    4,
                    self.world.get().performance_counters().step_time(),
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
    for (_, _, manifold) in world.collision_world().contact_manifolds() {
        for c in manifold.contacts() {
            if existing.contains_key(&c.id) {
                if running {
                    existing.insert(c.id, true);
                }
            } else {
                existing.insert(c.id, false);
            }

            let color = if existing[&c.id] {
                Point3::new(0.0, 0.0, 1.0)
            } else {
                Point3::new(1.0, 0.0, 0.0)
            };

            window.draw_planar_line(&c.contact.world1, &c.contact.world2, &color);

            let center = na::center(&c.contact.world1, &c.contact.world2);
            let end = center + *c.contact.normal * 0.4f32;
            window.draw_planar_line(&center, &end, &Point3::new(0.0, 1.0, 1.0))
        }
    }
}
