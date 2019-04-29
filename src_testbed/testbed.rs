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
use kiss3d::conrod;
use na::{self, Point2, Point3, Vector3};
use ncollide::query::{self, Ray};
use ncollide::utils::GenerationalId;
use ncollide::world::CollisionGroups;
use nphysics::joint::{ConstraintHandle, MouseConstraint};
use nphysics::object::{BodyHandle, BodyPartHandle, ColliderHandle, ColliderAnchor, ActivationStatus};
use nphysics::world::World;
use nphysics::math::ForceType;
use crate::world_owner::WorldOwner;
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
        const TIME_OF_IMPACT = 1 << 1;
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
    pub grabbed_object: Option<BodyPartHandle>,
    pub grabbed_object_constraint: Option<ConstraintHandle>,
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
    world: Box<WorldOwner>,
    window: Option<Box<Window>>,
    graphics: GraphicsManager,
    nsteps: usize,
    callbacks: Callbacks,
    time: f32,
    hide_counters: bool,
    persistant_contacts: HashMap<GenerationalId, bool>,
    font: Rc<Font>,
    cursor_pos: Point2<f32>,
    ui: TestbedUi,
    state: TestbedState
}

type Callbacks = Vec<Box<Fn(&mut WorldOwner, &mut GraphicsManager, f32)>>;

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let world = World::new();

        let mut window = Box::new(Window::new("nphysics: 3d demo"));
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_framerate_limit(Some(60));
        window.set_light(Light::StickToCamera);

        let mut flags = TestbedStateFlags::SLEEP;

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
            world: Box::new(Arc::new(RwLock::new(world))),
            callbacks: Vec::new(),
            window: Some(window),
            graphics,
            nsteps: 1,
            time: 0.0,
            hide_counters: true,
            persistant_contacts: HashMap::new(),
            font: Font::default(),
            cursor_pos: Point2::new(0.0f32, 0.0),
            ui,
            state,
        }
    }

    pub fn new(world: World<f32>) -> Self {
        Self::new_with_world_owner(Box::new(world))
    }

    pub fn new_with_world_owner(world: Box<WorldOwner>) -> Self {
        let mut res = Testbed::new_empty();

        res.set_world_owner(world);
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

    pub fn hide_performance_counters(&mut self) {
        self.hide_counters = true;
    }

    pub fn show_performance_counters(&mut self) {
        self.hide_counters = false;
    }

    pub fn set_world(&mut self, world: World<f32>) {
        self.set_world_owner(Box::new(world))
    }

    pub fn set_builders(&mut self, builders: Vec<(&'static str, fn(&mut Self))>) {
        self.state.example_names = builders.iter().map(|e| e.0).collect();
        self.builders = builders
    }

    pub fn set_world_owner(&mut self, world: Box<WorldOwner>) {
        let params = {
            let prev_world = self.world.get();
            prev_world.integration_parameters().clone()
        };

        let dt = params.dt();
        self.world = world;
        let mut world = self.world.get_mut();
        *world.integration_parameters_mut() = params;
        world.enable_performance_counters();
        world.set_timestep(0.0); // Update the internal state so that we can directly visualize things lake AABBs.
        world.step();
        world.set_timestep(dt);
        self.state.action_flags.set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        self.graphics.look_at(at, zoom);
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.graphics.look_at(eye, at);
    }

    pub fn set_body_color(&mut self, body: BodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(body, color);
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

    pub fn add_callback<F: Fn(&mut WorldOwner, &mut GraphicsManager, f32) + 'static>(
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
                let physics_world = &mut self.world.get_mut();

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
                    for (b, inter) in physics_world
                        .collider_world()
                        .interferences_with_ray(&ray, &all_groups)
                        {
                            if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                mintoi = inter.toi;

                                let subshape = b.shape().subshape_containing_feature(inter.feature);
                                minb = Some(b.body_part(subshape));
                            }
                        }

                    if let Some(body_part) = minb {
                        if modifier.contains(Modifiers::Control) {
                            if !body_part.is_ground() {
                                self.graphics.remove_body_nodes(window, body_part.0);
                                physics_world.remove_bodies(&[body_part.0]);
                            }
                        } else {
                            physics_world.body_mut(body_part.0)
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
                    for (b, inter) in physics_world
                        .collider_world()
                        .interferences_with_ray(&ray, &all_groups)
                        {
                            if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                mintoi = inter.toi;

                                let subshape = b.shape().subshape_containing_feature(inter.feature);
                                minb = Some(b.body_part(subshape));
                            }
                        }

                    if let Some(body_part_handle) = minb {
                        if physics_world.body(body_part_handle.0).unwrap().status_dependent_ndofs() != 0 {
                            self.state.grabbed_object = minb;
                            for n in self
                                .graphics
                                .body_nodes_mut(body_part_handle.0)
                                .unwrap()
                                .iter_mut()
                                {
                                    if let Some(joint) = self.state.grabbed_object_constraint {
                                        physics_world.remove_constraint(joint);
                                    }

                                    let attach1 = ray.origin + ray.dir * mintoi;
                                    let attach2 = {
                                        let body = physics_world.body(body_part_handle.0).unwrap();
                                        let part = body.part(body_part_handle.1).unwrap();
                                        body.material_point_at_world_point(part, &attach1)
                                    };
                                    let constraint = MouseConstraint::new(
                                        BodyPartHandle::ground(),
                                        body_part_handle,
                                        attach1,
                                        attach2,
                                        1.0,
                                    );
                                    self.state.grabbed_object_plane = (attach1, -ray.dir);
                                    self.state.grabbed_object_constraint =
                                        Some(physics_world.add_constraint(constraint));
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
                    let physics_world = &mut self.world.get_mut();
                    let _ = physics_world.remove_constraint(joint);
                }

                self.state.grabbed_object = None;
                self.state.grabbed_object_constraint = None;
            }
            WindowEvent::CursorPos(x, y, modifiers) => {
                let physics_world = &mut self.world.get_mut();

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
                        let joint = physics_world
                            .constraint_mut(joint)
                            .downcast_mut::<MouseConstraint<f32>>()
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
        {
            let mut world = self.world.get_mut();
            self.ui.update(window, world.deref_mut(), &mut self.state);
        }

        // Handle UI actions.
        {
            let example_changed = self.state.action_flags.contains(TestbedActionFlags::EXAMPLE_CHANGED);
            if example_changed {
                self.state.action_flags.set(TestbedActionFlags::EXAMPLE_CHANGED, false);
                self.clear(window);
                self.builders[self.state.selected_example].1(self);
            }

            let mut world = self.world.get_mut();

            if self.state.action_flags.contains(TestbedActionFlags::RESET_WORLD_GRAPHICS) {
                self.state.action_flags.set(TestbedActionFlags::RESET_WORLD_GRAPHICS, false);
                for co in world.colliders() {
                    self.graphics
                        .add(window, co.handle(), &world);
                }
            }

            if example_changed || self.state.prev_flags.contains(TestbedStateFlags::WIREFRAME) !=
                self.state.flags.contains(TestbedStateFlags::WIREFRAME) {

                for co in world.colliders() {
                    if let Some(ns) = self.graphics.body_nodes_mut(co.body())
                    {
                        for n in self.graphics.nodes_mut() {
                            let is_sensor = if let Some(collider) = world.collider(n.collider()) {
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
                    for body in world.bodies_mut() {
                        body.set_deactivation_threshold(Some(ActivationStatus::default_threshold()))
                    }
                } else {
                    for body in world.bodies_mut() {
                        body.activate();
                        body.set_deactivation_threshold(None)
                    }
                }
            }

            if self.state.prev_flags.contains(TestbedStateFlags::TIME_OF_IMPACT) !=
                self.state.flags.contains(TestbedStateFlags::TIME_OF_IMPACT) {
                unimplemented!()
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SUB_STEPPING) !=
                self.state.flags.contains(TestbedStateFlags::SUB_STEPPING) {
                unimplemented!()
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
                    self.graphics.show_aabbs(&mut world, window)
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

        for mut event in window.events().iter() {
            let event = self.handle_common_event(event);
            self.handle_special_event(window, event);
        }

        /*
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::MouseButton(_, Action::Press, modifier) => {
                    let physics_world = &mut self.world.get_mut();

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
                        for (b, inter) in physics_world
                            .collider_world()
                            .interferences_with_ray(&ray, &all_groups)
                            {
                                if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                    mintoi = inter.toi;

                                    let subshape = b.shape().subshape_containing_feature(inter.feature);
                                    minb = Some(b.body_part(subshape));
                                }
                            }

                        if let Some(body_part) = minb {
                            if modifier.contains(Modifiers::Control) {
                                if !body_part.is_ground() {
                                    self.graphics.remove_body_nodes(window, body_part.0);
                                    physics_world.remove_bodies(&[body_part.0]);
                                }
                            } else {
                                physics_world.body_mut(body_part.0)
                                    .unwrap()
                                    .apply_force_at_point(body_part.1,
                                                          &(ray.dir.normalize() * 0.01),
                                                          &ray.point_at(mintoi),
                                                          ForceType::Impulse,
                                                true);
                            }
                        }

                        event.inhibited = true;
                    } else if modifier.contains(Modifiers::Control) {
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
                        for (b, inter) in physics_world
                            .collider_world()
                            .interferences_with_ray(&ray, &all_groups)
                            {
                                if !b.query_type().is_proximity_query() && inter.toi < mintoi {
                                    mintoi = inter.toi;

                                    let subshape = b.shape().subshape_containing_feature(inter.feature);
                                    minb = Some(b.body_part(subshape));
                                }
                            }

                        if let Some(body_part_handle) = minb {
                            if physics_world.body(body_part_handle.0).unwrap().status_dependent_ndofs() != 0 {
                                self.state.grabbed_object = minb;
                                for n in self
                                    .graphics
                                    .body_nodes_mut(body_part_handle.0)
                                    .unwrap()
                                    .iter_mut()
                                    {
                                        if let Some(joint) = self.state.grabbed_object_constraint {
                                            physics_world.remove_constraint(joint);
                                        }

                                        let attach1 = ray.origin + ray.dir * mintoi;
                                        let attach2 = {
                                            let body = physics_world.body(body_part_handle.0).unwrap();
                                            let part = body.part(body_part_handle.1).unwrap();
                                            body.material_point_at_world_point(part, &attach1)
                                        };
                                        let constraint = MouseConstraint::new(
                                            BodyPartHandle::ground(),
                                            body_part_handle,
                                            attach1,
                                            attach2,
                                            1.0,
                                        );
                                        self.state.grabbed_object_plane = (attach1, -ray.dir);
                                        self.state.grabbed_object_constraint =
                                            Some(physics_world.add_constraint(constraint));
                                        n.select()
                                    }
                            }
                        }

                        event.inhibited = true;
                    }
                }
                WindowEvent::MouseButton(_, Action::Release, _) => {
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
                        let physics_world = &mut self.world.get_mut();
                        let _ = physics_world.remove_constraint(joint);
                    }

                    self.state.grabbed_object = None;
                    self.state.grabbed_object_constraint = None;
                }
                WindowEvent::CursorPos(x, y, modifiers) => {
                    let physics_world = &mut self.world.get_mut();

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
                        query::ray_internal::plane_toi_with_ray(ppos, pdir, &Ray::new(pos, dir))
                            {
                                let joint = physics_world
                                    .constraint_mut(joint)
                                    .downcast_mut::<MouseConstraint<f32>>()
                                    .unwrap();
                                joint.set_anchor_1(pos + dir * inter)
                            }
                    }

                    event.inhibited = modifiers.contains(Modifiers::Control)
                        || modifiers.contains(Modifiers::Shift);
                }
                //         WindowEvent::Key(Key::Tab, Action::Release, _) => self.graphics.switch_cameras(),
                WindowEvent::Key(Key::T, Action::Release, _) => {
                    if self.state.running == RunMode::Stop {
                        self.state.running = RunMode::Running;
                    } else {
                        self.state.running = RunMode::Stop;
                    }
                }
                WindowEvent::Key(Key::S, Action::Release, _) => self.state.running = RunMode::Step,
                //         WindowEvent::MouseButton(MouseButton::Button2, Action::Press, Key::LControl) |
                //         WindowEvent::MouseButton(MouseButton::Button2, Action::Press, Key::RControl)  => {
                //             let mut graphics = self.graphics;
                //             let geom   = Cuboid::new(Vector3::new(0.5f32, 0.5f32, 0.5f32));
                //             let mut rb = RigidBody::new_dynamic(geom, 4.0f32, 0.3, 0.6);

                //             let size = window.size();
                //             let (pos, dir) = graphics.camera().unproject(&cursor_pos, &size);

                //             rb.set_translation(Translation3::from(pos.coords));
                //             rb.set_lin_vel(dir * 1000.0f32);

                //             let body = self.world.add_rigid_body(rb);
                //             self.world.add_ccd_to(&body, 1.0, false);
                //             graphics.add(window, WorldObject::RigidBody(body));
                //         },
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

                //          let body = physics_world.add_rigid_body(rb);
                //          graphics.add(window, body, &physics_world.rigid_bodies());
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

                //          let body = physics_world.add_rigid_body(rb);
                //          graphics.add(window, body, &self.world.rigid_bodies());
                //      }
                _ => {}
            }
        }
        */

        if self.state.running != RunMode::Stop {
            // let before = time::precise_time_s();
            for _ in 0..self.nsteps {
                for f in &self.callbacks {
                    f(&mut *self.world, &mut self.graphics, self.time)
                }

                let mut world = self.world.get_mut();
                world.step();
                if !self.hide_counters {
                    #[cfg(not(feature = "log"))]
                    println!("{}", world.performance_counters());
                    #[cfg(feature = "log")]
                    debug!("{}", world.performance_counters());
                }
                self.time += world.timestep();
            }
        }

        self.graphics.draw(&self.world.get(), window);

        if self.state.flags.contains(TestbedStateFlags::CONTACT_POINTS) {
            draw_collisions(
                window,
                &self.world.get(),
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
            let world = self.world.get();
            let counters = world.performance_counters();

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

const CONTROLS: &str = "Controls:
    Ctrl + click + drag: select and move a solid.
    Left click + drag: rotate the camera.
    Right click + drag: pan the camera.
    Mouse wheel: zoom in/zoom out.";

fn draw_collisions(
    window: &mut Window,
    world: &World<f32>,
    existing: &mut HashMap<GenerationalId, bool>,
    running: bool,
) {
    for (_, _, _, manifold) in world.collider_world().contact_pairs(false) {
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
