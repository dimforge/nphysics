#[cfg(feature = "dim3")]
use num::Bounded;
use std::collections::HashMap;
use std::env;
use std::mem;
use std::path::Path;
use std::rc::Rc;
use std::sync::{Arc, RwLock};

use crate::engine::{GraphicsWindow, GraphicsManager};
use kiss3d::event::Event;
use kiss3d::camera::Camera;
use kiss3d::event::{Action, Key, Modifiers, WindowEvent, MouseButton};
use kiss3d::light::Light;
use kiss3d::loader::obj;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use na::{self, Point2, Point3, Vector3};
#[cfg(feature = "dim3")]
use ncollide::query;
use ncollide::query::{ContactId, Ray};
use ncollide::pipeline::object::CollisionGroups;
use nphysics::force_generator::DefaultForceGeneratorSet;
use nphysics::joint::{DefaultJointConstraintHandle, MouseConstraint, DefaultJointConstraintSet};
#[cfg(feature = "dim2")]
use nphysics::object::ColliderAnchor;
use nphysics::object::{DefaultBodyHandle, BodyPartHandle, DefaultBodyPartHandle,  DefaultColliderHandle, ActivationStatus, DefaultBodySet, DefaultColliderSet};
use nphysics::world::{DefaultDynamicWorld, DefaultColliderWorld};
#[cfg(feature = "dim3")]
use nphysics::math::ForceType;
use crate::ui::TestbedUi;


#[derive(PartialEq)]
pub enum RunMode {
    Running,
    Stop,
    Step,
    Quit
}

#[cfg(not(feature = "log"))]
fn usage(exe_name: &str) {
    println!("Usage: {} [OPTION] ", exe_name);
    println!();
    println!("Options:");
    println!("    --help  - prints this help message and exits.");
    println!("    --pause - do not start the simulation right away.");
}

#[cfg(feature = "log")]
fn usage(exe_name: &str) {
    info!("Usage: {} [OPTION] ", exe_name);
    info!("");
    info!("Options:");
    info!("    --help  - prints this help message and exits.");
    info!("    --pause - do not start the simulation right away.");
}

bitflags! {
    #[derive(Default)]
    pub struct TestbedStateFlags: u32 {
        const NONE = 0;
        const SLEEP = 1 << 0;
        const CCD = 1 << 1;
        const SUB_STEPPING = 1 << 2;
        const SHAPES = 1 << 3;
        const JOINTS = 1 << 4;
        const AABBS = 1 << 5;
        const CONTACT_POINTS = 1 << 6;
        const CONTACT_NORMALS = 1 << 7;
        const CENTER_OF_MASSES = 1 << 8;
        const WIREFRAME = 1 << 9;
        const STATISTICS = 1 << 10;
        const PROFILE = 1 << 11;
    }
}

bitflags! {
    pub struct TestbedActionFlags: u32 {
        const RESET_WORLD_GRAPHICS = 1 << 0;
        const EXAMPLE_CHANGED = 1 << 1;
    }
}

pub struct TestbedState {
    pub running: RunMode,
    pub draw_colls: bool,
    pub grabbed_object: Option<DefaultBodyPartHandle>,
    pub grabbed_object_constraint: Option<DefaultJointConstraintHandle>,
    pub grabbed_object_plane: (Point3<f32>, Vector3<f32>),
    pub drawing_ray: Option<Point2<f32>>,
    pub prev_flags: TestbedStateFlags,
    pub flags: TestbedStateFlags,
    pub action_flags: TestbedActionFlags,
    pub example_names: Vec<&'static str>,
    pub selected_example: usize
}

pub struct Testbed {
    builders: Vec<(&'static str, fn(&mut Testbed))>,
    dynamic_world: DefaultDynamicWorld<f32>,
    collider_world: DefaultColliderWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    forces: DefaultForceGeneratorSet<f32>,
    constraints: DefaultJointConstraintSet<f32>,
    window: Option<Box<Window>>,
    graphics: GraphicsManager,
    nsteps: usize,
    callbacks: Callbacks,
    time: f32,
    hide_counters: bool,
    persistant_contacts: HashMap<ContactId, bool>,
    font: Rc<Font>,
    cursor_pos: Point2<f32>,
    ground_handle: Option<DefaultBodyHandle>,
    ui: TestbedUi,
    state: TestbedState
}

type Callbacks = Vec<Box<Fn(
    &mut DefaultDynamicWorld<f32>,
    &mut DefaultColliderWorld<f32>,
    &mut DefaultBodySet<f32>,
    &mut DefaultColliderSet<f32>,
    &mut GraphicsManager,
    f32
)>>;

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();

        let mut window = Box::new(Window::new("nphysics: 3d demo"));
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_framerate_limit(Some(60));
        window.set_light(Light::StickToCamera);

        let flags = TestbedStateFlags::SLEEP | TestbedStateFlags::CCD;

        let ui = TestbedUi::new(&mut window);
        let state = TestbedState {
            running: RunMode::Running,
            draw_colls: false,
            grabbed_object: None,
            grabbed_object_constraint: None,
            grabbed_object_plane: (Point3::origin(), na::zero()),
            drawing_ray: None,
            prev_flags: flags,
            flags,
            action_flags: TestbedActionFlags::empty(),
            example_names: Vec::new(),
            selected_example: 0,
        };

        Testbed {
            builders: Vec::new(),
            dynamic_world: DefaultDynamicWorld::new(Vector3::zeros()),
            collider_world: DefaultColliderWorld::new(),
            bodies: DefaultBodySet::new(),
            colliders: DefaultColliderSet::new(),
            forces: DefaultForceGeneratorSet::new(),
            constraints: DefaultJointConstraintSet::new(),
            callbacks: Vec::new(),
            window: Some(window),
            graphics,
            nsteps: 1,
            time: 0.0,
            hide_counters: true,
            persistant_contacts: HashMap::new(),
            font: Font::default(),
            cursor_pos: Point2::new(0.0f32, 0.0),
            ground_handle: None,
            ui,
            state,
        }
    }

    pub fn new(dynamic_world: DefaultDynamicWorld<f32>,
               collider_world: DefaultColliderWorld<f32>,
               bodies: DefaultBodySet<f32>,
               colliders: DefaultColliderSet<f32>,
               constraints: DefaultJointConstraintSet<f32>,
               forces: DefaultForceGeneratorSet<f32>)
        -> Self {
        let mut res = Self::new_empty();
        res.set_world(dynamic_world, collider_world, bodies, colliders, constraints, forces);
        res
    }

    pub fn from_builders(default: usize, builders: Vec<(&'static str, fn(&mut Self))>) -> Self {
        let mut res = Testbed::new_empty();
        res.state.action_flags.set(TestbedActionFlags::EXAMPLE_CHANGED, true);
        res.state.selected_example = default;
        res.set_builders(builders);
        res
    }

    pub fn set_number_of_steps_per_frame(&mut self, nsteps: usize) {
        self.nsteps = nsteps
    }

    pub fn set_ground_handle(&mut self, handle: Option<DefaultBodyHandle>) {
        self.ground_handle = handle;
        self.graphics.set_ground_handle(handle);
    }

    pub fn hide_performance_counters(&mut self) {
        self.hide_counters = true;
    }

    pub fn show_performance_counters(&mut self) {
        self.hide_counters = false;
    }

    pub fn set_world(&mut self,
                     dynamic_world: DefaultDynamicWorld<f32>,
                     collider_world: DefaultColliderWorld<f32>,
                     bodies: DefaultBodySet<f32>,
                     colliders: DefaultColliderSet<f32>,
                     constraints: DefaultJointConstraintSet<f32>,
                     forces: DefaultForceGeneratorSet<f32>) {
        self.dynamic_world = dynamic_world;
        self.collider_world = collider_world;
        self.bodies = bodies;
        self.colliders = colliders;
        self.constraints = constraints;
        self.forces = forces;
        self.state.action_flags.set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
        self.collider_world.maintain(&mut self.bodies, &mut self.colliders);
    }

    pub fn set_builders(&mut self, builders: Vec<(&'static str, fn(&mut Self))>) {
        self.state.example_names = builders.iter().map(|e| e.0).collect();
        self.builders = builders
    }

//    pub fn set_world_owner(&mut self, world: Box<WorldOwner>) {
//        let parameters = {
//            let prev_world = self.world.get();
//            prev_world.parameters().clone()
//        };
//
//        let dt = parameters.dt();
//        self.world = world;
//        let mut world = self.world.get_mut();
//        *world.parameters = parameters;
//        world.enable_performance_counters();
//        world.set_timestep(0.0); // Update the internal state so that we can directly visualize things lake AABBs.
//        world.step();
//        world.set_timestep(dt);
//        self.state.action_flags.set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
//    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        self.graphics.look_at(at, zoom);
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.graphics.look_at(eye, at);
    }

    pub fn set_body_color(&mut self, body: DefaultBodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(body, color);
    }

    pub fn set_collider_color(&mut self, collider: DefaultColliderHandle, color: Point3<f32>) {
        self.graphics.set_collider_color(collider, color);
    }

//    pub fn world(&self) -> &Box<WorldOwner> {
//        &self.world
//    }

    pub fn graphics_mut(&mut self) -> &mut GraphicsManager {
        &mut self.graphics
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path = Path::new(path);
        let empty = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").expect("Unable to open the obj file.");

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

    fn clear(&mut self, window: &mut Window) {
        self.callbacks.clear();
        self.persistant_contacts.clear();
        self.state.grabbed_object = None;
        self.state.grabbed_object_constraint = None;
        self.graphics.clear(window);
    }

    pub fn add_callback<F: Fn(&mut DefaultDynamicWorld<f32>,
                              &mut DefaultColliderWorld<f32>,
                              &mut DefaultBodySet<f32>,
                              &mut DefaultColliderSet<f32>,
                              &mut GraphicsManager, f32) + 'static>(
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
                    self.state.running = RunMode::Stop;
                }
            }
        }

        let window = mem::replace(&mut self.window, None).unwrap();
        window.render_loop(self);
    }

    fn handle_common_event<'b>(&mut self, event: Event<'b>) -> Event<'b> {
        match event.value {
            WindowEvent::Key(Key::T, Action::Release, _) => {
                if self.state.running == RunMode::Stop {
                    self.state.running = RunMode::Running;
                } else {
                    self.state.running = RunMode::Stop;
                }
            }
            WindowEvent::Key(Key::S, Action::Release, _) => self.state.running = RunMode::Step,
            WindowEvent::Key(Key::R, Action::Release, _) => {
                self.state.action_flags.set(TestbedActionFlags::EXAMPLE_CHANGED, true)
            },
            _ => {}
        }

        event
    }

    #[cfg(feature = "dim2")]
    fn handle_special_event(&mut self, window: &mut Window, mut event: Event) {
        if window.is_conrod_ui_capturing_mouse() {
            return;
        }

        match event.value {
            WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                let physics_world = &mut self.world.get_mut();
                let all_groups = &CollisionGroups::new();
                for b in physics_world
                    .collider_world()
                    .interferences_with_point(&self.cursor_pos, all_groups)
                    {
                        if !b.query_type().is_proximity_query() && !b.body().is_ground() {

                            if
                            let ColliderAnchor::OnBodyPart { body_part, .. } = b.anchor()
                            {
                                self.state.grabbed_object = Some(*body_part);
                            } else { continue; }
                        }
                    }

                if modifier.contains(Modifiers::Shift) {
                    if let Some(body_part) = self.state.grabbed_object {
                        if !body_part.is_ground() {
                            self.graphics.remove_body_nodes(window, body_part.0);
                            physics_world.remove_bodies(&[body_part.0]);
                        }
                    }

                    self.state.grabbed_object = None;
                } else if !modifier.contains(Modifiers::Control) {
                    if let Some(body) = self.state.grabbed_object {
                        if let Some(joint) = self.state.grabbed_object_constraint {
                            let _ = physics_world.remove_constraint(joint);
                        }

                        let body_pos = physics_world.body(body.0).unwrap().part(body.1).unwrap().position();
                        let attach1 = self.cursor_pos;
                        let attach2 = body_pos.inverse() * attach1;
                        let joint = MouseConstraint::new(
                            BodyPartHandle::ground(),
                            body,
                            attach1,
                            attach2,
                            1.0,
                        );
                        self.state.grabbed_object_constraint =
                            Some(physics_world.add_constraint(joint));

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
                    self.state.drawing_ray = Some(self.cursor_pos);
                } else {
                    self.state.grabbed_object = None;
                }
            }
            WindowEvent::MouseButton(MouseButton::Button1, Action::Release, _) => {
                let physics_world = &mut self.world.get_mut();
                if let Some(body) = self.state.grabbed_object {
                    for n in self
                        .graphics
                        .body_nodes_mut(body.0)
                        .unwrap()
                        .iter_mut()
                        {
                            n.unselect()
                        }
                }

                if let Some(joint) = self.state.grabbed_object_constraint {
                    let _ = physics_world.remove_constraint(joint);
                }


                if let Some(start) = self.state.drawing_ray {
                    self.graphics.add_ray(Ray::new(start, self.cursor_pos - start));
                }

                self.state.drawing_ray = None;
                self.state.grabbed_object = None;
                self.state.grabbed_object_constraint = None;
            }
            WindowEvent::CursorPos(x, y, modifiers) => {
                let physics_world = &mut self.world.get_mut();
                self.cursor_pos.x = x as f32;
                self.cursor_pos.y = y as f32;

                self.cursor_pos = self
                    .graphics
                    .camera()
                    .unproject(&self.cursor_pos, &na::convert(window.size()));

                let attach2 = self.cursor_pos;
                if self.state.grabbed_object.is_some() {
                    let joint = self.state.grabbed_object_constraint.unwrap();
                    let joint = physics_world
                        .constraint_mut(joint)
                        .downcast_mut::<MouseConstraint<f32>>()
                        .unwrap();
                    joint.set_anchor_1(attach2);
                }

                event.inhibited = modifiers.contains(Modifiers::Control)
                    || modifiers.contains(Modifiers::Shift);
            }
            _ => {}
        }
    }

    #[cfg(feature = "dim3")]
    fn handle_special_event(&mut self, window: &mut Window, mut event: Event) {
        if window.is_conrod_ui_capturing_mouse() {
            return;
        }

        match event.value {
            WindowEvent::MouseButton(MouseButton::Button1, Action::Press, modifier) => {
                if modifier.contains(Modifiers::Alt) {
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(size));
                    let ray = Ray::new(pos, dir);
                    self.graphics.add_ray(ray);

                    event.inhibited = true;
                } else if modifier.contains(Modifiers::Shift) {
                    // XXX: huge and uggly code duplication for the ray cast.
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(size));
                    let ray = Ray::new(pos, dir);

                    // cast the ray
                    let mut mintoi = Bounded::max_value();
                    let mut minb = None;

                    let all_groups = CollisionGroups::new();
                    for (b, inter) in self.collider_world
                        .interferences_with_ray(&self.colliders, &ray, &all_groups)
                        {
                            if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                mintoi = inter.toi;

                                let subshape = b.shape().subshape_containing_feature(inter.feature);
                                minb = Some(b.body_part(subshape));
                            }
                        }

                    if let Some(body_part) = minb {
                        if modifier.contains(Modifiers::Control) {
                            if Some(body_part.0) != self.ground_handle {
                                self.graphics.remove_body_nodes(window, body_part.0);
                                self.bodies.remove(body_part.0);
                            }
                        } else {
                            self.bodies.get_mut(body_part.0)
                                .unwrap()
                                .apply_force_at_point(body_part.1,
                                                      &(ray.dir.normalize() * 0.01),
                                                      &ray.point_at(mintoi),
                                                      ForceType::Impulse,
                                                      true);
                        }
                    }

                    event.inhibited = true;
                } else if !modifier.contains(Modifiers::Control) {
                    match self.state.grabbed_object {
                        Some(body) => for n in self
                            .graphics
                            .body_nodes_mut(body.0)
                            .unwrap()
                            .iter_mut()
                            {
                                n.unselect()
                            },
                        None => {}
                    }

                    // XXX: huge and uggly code duplication for the ray cast.
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(size));
                    let ray = Ray::new(pos, dir);

                    // cast the ray
                    let mut mintoi = Bounded::max_value();
                    let mut minb = None;

                    let all_groups = CollisionGroups::new();
                    for (b, inter) in self
                        .collider_world
                        .interferences_with_ray(&self.colliders, &ray, &all_groups)
                        {
                            if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                mintoi = inter.toi;

                                let subshape = b.shape().subshape_containing_feature(inter.feature);
                                minb = Some(b.body_part(subshape));
                            }
                        }

                    if let Some(body_part_handle) = minb {
                        if self.bodies.get(body_part_handle.0).unwrap().status_dependent_ndofs() != 0 {
                            self.state.grabbed_object = minb;
                            for n in self
                                .graphics
                                .body_nodes_mut(body_part_handle.0)
                                .unwrap()
                                .iter_mut()
                                {
                                    if let Some(joint) = self.state.grabbed_object_constraint {
                                        let constraint = self.constraints.remove(joint).unwrap();
                                        let (b1, b2) = constraint.anchors();
                                        self.bodies.get_mut(b1.0).unwrap().activate();
                                        self.bodies.get_mut(b2.0).unwrap().activate();
                                    }

                                    let attach1 = ray.origin + ray.dir * mintoi;
                                    let attach2 = {
                                        let body = self.bodies.get_mut(body_part_handle.0).unwrap();
                                        body.activate();
                                        let part = body.part(body_part_handle.1).unwrap();
                                        body.material_point_at_world_point(part, &attach1)
                                    };

                                    /*
                                    let constraint = MouseConstraint::new(
                                        BodyPartHandle::ground(),
                                        body_part_handle,
                                        attach1,
                                        attach2,
                                        1.0,
                                    );
                                    self.state.grabbed_object_plane = (attach1, -ray.dir);
                                    self.state.grabbed_object_constraint = Some(self.constraints.insert(Box::new(constraint)));
                                    */
                                    n.select()
                                }
                        }
                    }

                    event.inhibited = true;
                }
            }
            WindowEvent::MouseButton(MouseButton::Button1, Action::Release, _) => {
                if let Some(body_part) = self.state.grabbed_object {
                    for n in self
                        .graphics
                        .body_nodes_mut(body_part.0)
                        .unwrap()
                        .iter_mut()
                        {
                            n.unselect()
                        }
                }

                if let Some(joint) = self.state.grabbed_object_constraint {
                    let constraint = self.constraints.remove(joint).unwrap();
                    let (b1, b2) = constraint.anchors();
                    self.bodies.get_mut(b1.0).unwrap().activate();
                    self.bodies.get_mut(b2.0).unwrap().activate();
                }

                self.state.grabbed_object = None;
                self.state.grabbed_object_constraint = None;
            }
            WindowEvent::CursorPos(x, y, modifiers) => {
                self.cursor_pos.x = x as f32;
                self.cursor_pos.y = y as f32;

                // update the joint
                if let Some(joint) = self.state.grabbed_object_constraint {
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&self.cursor_pos, &na::convert(size));
                    let (ref ppos, ref pdir) = self.state.grabbed_object_plane;

                    if let Some(inter) =
                    query::ray_toi_with_plane(ppos, pdir, &Ray::new(pos, dir))
                    {
                        let joint = self.constraints
                            .get_mut(joint)
                            .unwrap()
                            .downcast_mut::<MouseConstraint<f32, DefaultBodyHandle>>()
                            .unwrap();
                        joint.set_anchor_1(pos + dir * inter)
                    }
                }

                event.inhibited = modifiers.contains(Modifiers::Shift);
            }
            _ => {}
        }
    }
}

type CameraEffects<'a> = (
    Option<&'a mut Camera>,
    Option<&'a mut PlanarCamera>,
    Option<&'a mut PostProcessingEffect>,
);

impl State for Testbed {
    fn cameras_and_effect(&mut self) -> CameraEffects<'_> {
         #[cfg(feature = "dim2")]
            let result = (None, Some(self.graphics.camera_mut() as &mut PlanarCamera), None);
        #[cfg(feature = "dim3")]
            let result = (Some(self.graphics.camera_mut() as &mut Camera), None, None);
        result
    }

    fn step(&mut self, window: &mut Window) {
        self.ui.update(window, &mut self.dynamic_world, &mut self.state);

        // Handle UI actions.
        {
            let example_changed = self.state.action_flags.contains(TestbedActionFlags::EXAMPLE_CHANGED);
            if example_changed {
                self.state.action_flags.set(TestbedActionFlags::EXAMPLE_CHANGED, false);
                self.clear(window);
                self.builders[self.state.selected_example].1(self);
            }

            if self.state.action_flags.contains(TestbedActionFlags::RESET_WORLD_GRAPHICS) {
                self.state.action_flags.set(TestbedActionFlags::RESET_WORLD_GRAPHICS, false);
                for (handle, _) in self.colliders.iter() {
                    self.graphics.add(window, handle, &self.colliders);
                }
            }

            if example_changed || self.state.prev_flags.contains(TestbedStateFlags::WIREFRAME) !=
                self.state.flags.contains(TestbedStateFlags::WIREFRAME) {

                for (_, co) in self.colliders.iter() {
                    if self.graphics.body_nodes_mut(co.body()).is_some() {
                        for n in self.graphics.nodes_mut() {
                            let is_sensor = if let Some(collider) = self.colliders.get(n.collider()) {
                                collider.is_sensor()
                            } else {
                                false
                            };

                            if let Some(node) = n.scene_node_mut() {
                                if is_sensor || self.state.flags.contains(TestbedStateFlags::WIREFRAME) {
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

            if self.state.prev_flags.contains(TestbedStateFlags::SLEEP) !=
                self.state.flags.contains(TestbedStateFlags::SLEEP) {
                if self.state.flags.contains(TestbedStateFlags::SLEEP) {
                    for (_, body) in self.bodies.iter_mut() {
                        body.set_deactivation_threshold(Some(ActivationStatus::default_threshold()))
                    }
                } else {
                    for (_, body) in self.bodies.iter_mut() {
                        body.activate();
                        body.set_deactivation_threshold(None)
                    }
                }
            }

            if self.state.prev_flags.contains(TestbedStateFlags::CCD) !=
                self.state.flags.contains(TestbedStateFlags::CCD) {
                self.dynamic_world.parameters.ccd_enabled = self.state.flags.contains(TestbedStateFlags::CCD);
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SUB_STEPPING) !=
                self.state.flags.contains(TestbedStateFlags::SUB_STEPPING) {
                self.dynamic_world.parameters.substepping_enabled = self.state.flags.contains(TestbedStateFlags::SUB_STEPPING);
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SHAPES) !=
                self.state.flags.contains(TestbedStateFlags::SHAPES) {
                unimplemented!()
            }

            if self.state.prev_flags.contains(TestbedStateFlags::JOINTS) !=
                self.state.flags.contains(TestbedStateFlags::JOINTS) {
                unimplemented!()
            }

            if example_changed || self.state.prev_flags.contains(TestbedStateFlags::AABBS) !=
                self.state.flags.contains(TestbedStateFlags::AABBS) {
                if self.state.flags.contains(TestbedStateFlags::AABBS) {
                    self.graphics.show_aabbs(&self.collider_world, &self.colliders, window)
                } else {
                    self.graphics.hide_aabbs(window)
                }
            }

            if self.state.prev_flags.contains(TestbedStateFlags::CENTER_OF_MASSES) !=
                self.state.flags.contains(TestbedStateFlags::CENTER_OF_MASSES) {
                unimplemented!()
            }
        }

        self.state.prev_flags = self.state.flags;

        for event in window.events().iter() {
            let event = self.handle_common_event(event);
            self.handle_special_event(window, event);
        }

        if self.state.running != RunMode::Stop {
            // let before = time::precise_time_s();
            for _ in 0..self.nsteps {
                for f in &self.callbacks {
                    f(&mut self.dynamic_world, &mut self.collider_world, &mut self.bodies, &mut self.colliders, &mut self.graphics, self.time)
                }

                self.dynamic_world.step(
                    &mut self.collider_world,
                    &mut self.bodies,
                    &mut self.colliders,
                    &mut self.constraints,
                    &mut self.forces
                );
                if !self.hide_counters {
                    #[cfg(not(feature = "log"))]
                    println!("{}", self.dynamic_world.counters);
                    #[cfg(feature = "log")]
                    debug!("{}", self.dynamic_world.counters);
                }
                self.time += self.dynamic_world.timestep();
            }
        }

        self.graphics.draw(&self.collider_world, &self.colliders, window);

        if self.state.flags.contains(TestbedStateFlags::CONTACT_POINTS) {
            draw_collisions(
                window,
                &self.collider_world,
                &self.colliders,
                &mut self.persistant_contacts,
                self.state.running != RunMode::Stop,
            );
        }

        if self.state.running == RunMode::Step {
            self.state.running = RunMode::Stop;
        }

        if self.state.running == RunMode::Quit {
            window.close()
        }

        let color = Point3::new(0.0, 0.0, 0.0);

        if true {
            let counters = self.dynamic_world.counters;

            let profile = format!(
            r#"Total: {:.2}ms
Collision detection: {:.2}ms
|_ Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
Island computation: {:.2}ms
Solver: {:.2}ms
|_ Assembly: {:.2}ms
   Velocity resolution: {:.2}ms
   Position resolution: {:.2}ms"#,
            counters.step_time() * 1000.0,
            counters.collision_detection_time() * 1000.0,
            counters.broad_phase_time() * 1000.0,
            counters.narrow_phase_time() * 1000.0,
            counters.island_construction_time() * 1000.0,
            counters.solver_time() * 1000.0,
            counters.assembly_time() * 1000.0,
            counters.velocity_resolution_time() * 1000.0,
            counters.position_resolution_time() * 1000.0);

//            let stats = format!(
//                r#"Total: {:.2}ms
//Collision detection: {:.2}ms
//|_ Broad-phase: {:.2}ms
//   Narrow-phase: {:.2}ms
//Island computation: {:.2}ms
//Solver: {:.2}ms
//|_ Assembly: {:.2}ms
//   Velocity resolution: {:.2}ms
//   Position resolution: {:.2}ms"#,
//                counters.step_time() * 1000.0,
//                counters.collision_detection_time() * 1000.0,
//                counters.broad_phase_time() * 1000.0,
//                counters.narrow_phase_time() * 1000.0,
//                counters.island_construction_time() * 1000.0,
//                counters.solver_time() * 1000.0,
//                counters.assembly_time() * 1000.0,
//                counters.velocity_resolution_time() * 1000.0,
//                counters.position_resolution_time() * 1000.0);

            if self.state.flags.contains(TestbedStateFlags::PROFILE) {
                window.draw_text(
                    &profile,
                    &Point2::origin(),
                    45.0,
                    &self.font,
                    &color,
                );
            }
        } else {
            window.draw_text("Paused", &Point2::origin(), 60.0, &self.font, &color);
        }
//        window.draw_text(CONTROLS, &Point2::new(0.0, 75.0), 40.0, &self.font, &color);
    }
}

fn draw_collisions(
    window: &mut Window,
    collider_world: &DefaultColliderWorld<f32>,
    colliders: &DefaultColliderSet<f32>,
    existing: &mut HashMap<ContactId, bool>,
    running: bool,
) {
    for (_, _, _, _, _, manifold) in collider_world.contact_pairs(colliders, false) {
        for c in manifold.contacts() {
            existing
                .entry(c.id)
                .and_modify(|value| {
                    if running {
                        *value = true
                    }
                })
                .or_insert(false);

            let color = if c.contact.depth < 0.0 { // existing[&c.id] {
                Point3::new(0.0, 0.0, 1.0)
            } else {
                Point3::new(1.0, 0.0, 0.0)
            };

            window.draw_graphics_line(&(c.contact.world1), &c.contact.world2, &color);
        }
    }
}
