use alga::general::{SubsetOf, SupersetOf};
#[cfg(feature = "dim3")]
use num::Bounded;
use std::collections::HashMap;
use std::env;
use std::mem;
use std::path::Path;
use std::rc::Rc;

use crate::engine::{GraphicsManager, GraphicsWindow};
#[cfg(feature = "fluids")]
use crate::objects::FluidRenderingMode;
use crate::ui::TestbedUi;
use kiss3d::camera::Camera;
use kiss3d::event::Event;
use kiss3d::event::{Action, Key, Modifiers, MouseButton, WindowEvent};
use kiss3d::light::Light;
use kiss3d::loader::obj;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use na::{self, Point2, Point3, RealField, Vector3};
use ncollide::pipeline::CollisionGroups;
#[cfg(feature = "dim3")]
use ncollide::query;
use ncollide::query::{ContactId, Ray};
use nphysics::force_generator::DefaultForceGeneratorSet;
use nphysics::joint::{DefaultJointConstraintHandle, DefaultJointConstraintSet, MouseConstraint};
#[cfg(feature = "dim3")]
use nphysics::math::ForceType;
#[cfg(feature = "dim2")]
use nphysics::object::ColliderAnchor;
use nphysics::object::{
    ActivationStatus, BodyPartHandle, DefaultBodyHandle, DefaultBodyPartHandle, DefaultBodySet,
    DefaultColliderHandle, DefaultColliderSet,
};
use nphysics::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
#[cfg(feature = "fluids")]
use salva::{coupling::ColliderCouplingSet, object::FluidHandle, LiquidWorld};

#[cfg(feature = "box2d-backend")]
use crate::box2d_world::Box2dWorld;

const NPHYSICS_BACKEND: usize = 0;
#[cfg(feature = "box2d-backend")]
const BOX2D_BACKEND: usize = 1;

#[derive(PartialEq)]
pub enum RunMode {
    Running,
    Stop,
    Step,
    Quit,
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
        const SUB_STEPPING = 1 << 1;
        const SHAPES = 1 << 2;
        const JOINTS = 1 << 3;
        const AABBS = 1 << 4;
        const CONTACT_POINTS = 1 << 5;
        const CONTACT_NORMALS = 1 << 6;
        const CENTER_OF_MASSES = 1 << 7;
        const WIREFRAME = 1 << 8;
        const STATISTICS = 1 << 9;
        const PROFILE = 1 << 10;
    }
}

bitflags! {
    pub struct TestbedActionFlags: u32 {
        const RESET_WORLD_GRAPHICS = 1 << 0;
        const EXAMPLE_CHANGED = 1 << 1;
        const RESTART = 1 << 2;
        const BACKEND_CHANGED = 1 << 3;
    }
}

pub struct TestbedState<N: RealField> {
    pub running: RunMode,
    pub draw_colls: bool,
    pub grabbed_object: Option<DefaultBodyPartHandle>,
    pub grabbed_object_constraint: Option<DefaultJointConstraintHandle>,
    pub grabbed_object_plane: (Point3<N>, Vector3<N>),
    pub can_grab_behind_ground: bool,
    pub drawing_ray: Option<Point2<N>>,
    pub prev_flags: TestbedStateFlags,
    pub flags: TestbedStateFlags,
    pub action_flags: TestbedActionFlags,
    pub backend_names: Vec<&'static str>,
    pub example_names: Vec<&'static str>,
    pub selected_example: usize,
    pub selected_backend: usize,
}

#[cfg(feature = "fluids")]
struct FluidsState<N: RealField> {
    world: LiquidWorld<N>,
    coupling: ColliderCouplingSet<N, DefaultBodyHandle>,
}

pub struct Testbed<N: RealField = f32> {
    builders: Vec<(&'static str, fn(&mut Testbed<N>))>,
    #[cfg(feature = "fluids")]
    fluids: Option<FluidsState<N>>,
    mechanical_world: DefaultMechanicalWorld<N>,
    geometrical_world: DefaultGeometricalWorld<N>,
    bodies: DefaultBodySet<N>,
    colliders: DefaultColliderSet<N>,
    forces: DefaultForceGeneratorSet<N>,
    constraints: DefaultJointConstraintSet<N>,
    window: Option<Box<Window>>,
    graphics: GraphicsManager,
    nsteps: usize,
    camera_locked: bool, // Used so that the camera can remain the same before and after we change backend or press the restart button.
    callbacks: Callbacks<N>,
    #[cfg(feature = "fluids")]
    callbacks_fluids: CallbacksFluids<N>,
    time: N,
    hide_counters: bool,
    persistant_contacts: HashMap<ContactId, bool>,
    font: Rc<Font>,
    cursor_pos: Point2<N>,
    ground_handle: Option<DefaultBodyHandle>,
    ui: TestbedUi,
    state: TestbedState<N>,
    #[cfg(feature = "box2d-backend")]
    box2d: Option<Box2dWorld>,
}

type Callbacks<N> = Vec<
    Box<
        dyn FnMut(
            &mut DefaultMechanicalWorld<N>,
            &mut DefaultGeometricalWorld<N>,
            &mut DefaultBodySet<N>,
            &mut DefaultColliderSet<N>,
            &mut GraphicsManager,
            N,
        ),
    >,
>;

#[cfg(feature = "fluids")]
type CallbacksFluids<N> = Vec<
    Box<
        dyn FnMut(
            &mut LiquidWorld<N>,
            &mut ColliderCouplingSet<N, DefaultBodyHandle>,
            &mut DefaultMechanicalWorld<N>,
            &mut DefaultGeometricalWorld<N>,
            &mut DefaultBodySet<N>,
            &mut DefaultColliderSet<N>,
            &mut GraphicsManager,
            N,
        ),
    >,
>;

impl<N: RealField + SupersetOf<f32> + SubsetOf<f32>> Testbed<N> {
    pub fn new_empty() -> Self {
        let graphics = GraphicsManager::new();

        #[cfg(feature = "dim3")]
        let mut window = Box::new(Window::new("nphysics: 3d demo"));
        #[cfg(feature = "dim2")]
        let mut window = Box::new(Window::new("nphysics: 2d demo"));
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_framerate_limit(Some(60));
        window.set_light(Light::StickToCamera);

        let flags = TestbedStateFlags::SLEEP;
        let ui = TestbedUi::new(&mut window);

        #[allow(unused_mut)]
        let mut backend_names = vec!["nphysics"];
        #[cfg(feature = "box2d-backend")]
        backend_names.push("box2d");

        let state = TestbedState {
            running: RunMode::Running,
            draw_colls: false,
            grabbed_object: None,
            grabbed_object_constraint: None,
            grabbed_object_plane: (Point3::origin(), na::zero()),
            can_grab_behind_ground: false,
            drawing_ray: None,
            prev_flags: flags,
            flags,
            action_flags: TestbedActionFlags::empty(),
            backend_names,
            example_names: Vec::new(),
            selected_example: 0,
            selected_backend: NPHYSICS_BACKEND,
        };

        let mechanical_world = DefaultMechanicalWorld::new(na::zero());
        let geometrical_world = DefaultGeometricalWorld::new();
        let bodies = DefaultBodySet::new();
        let colliders = DefaultColliderSet::new();
        let forces = DefaultForceGeneratorSet::new();
        let constraints = DefaultJointConstraintSet::new();

        Testbed {
            builders: Vec::new(),
            #[cfg(feature = "fluids")]
            fluids: None,
            mechanical_world,
            geometrical_world,
            bodies,
            colliders,
            forces,
            constraints,
            callbacks: Vec::new(),
            #[cfg(feature = "fluids")]
            callbacks_fluids: Vec::new(),
            window: Some(window),
            graphics,
            nsteps: 1,
            camera_locked: false,
            time: N::zero(),
            hide_counters: true,
            persistant_contacts: HashMap::new(),
            font: Font::default(),
            cursor_pos: Point2::new(N::zero(), N::zero()),
            ground_handle: None,
            ui,
            state,
            #[cfg(feature = "box2d-backend")]
            box2d: None,
        }
    }

    pub fn new(
        mechanical_world: DefaultMechanicalWorld<N>,
        geometrical_world: DefaultGeometricalWorld<N>,
        bodies: DefaultBodySet<N>,
        colliders: DefaultColliderSet<N>,
        constraints: DefaultJointConstraintSet<N>,
        forces: DefaultForceGeneratorSet<N>,
    ) -> Self {
        let mut res = Self::new_empty();
        res.set_world(
            mechanical_world,
            geometrical_world,
            bodies,
            colliders,
            constraints,
            forces,
        );
        res
    }

    pub fn from_builders(default: usize, builders: Vec<(&'static str, fn(&mut Self))>) -> Self {
        let mut res = Testbed::new_empty();
        res.state
            .action_flags
            .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
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

    pub fn allow_grabbing_behind_ground(&mut self, allow: bool) {
        self.state.can_grab_behind_ground = allow;
    }

    pub fn hide_performance_counters(&mut self) {
        self.hide_counters = true;
    }

    pub fn show_performance_counters(&mut self) {
        self.hide_counters = false;
    }

    pub fn set_world(
        &mut self,
        mut mechanical_world: DefaultMechanicalWorld<N>,
        geometrical_world: DefaultGeometricalWorld<N>,
        bodies: DefaultBodySet<N>,
        colliders: DefaultColliderSet<N>,
        joint_constraints: DefaultJointConstraintSet<N>,
        force_generators: DefaultForceGeneratorSet<N>,
    ) {
        mechanical_world.integration_parameters =
            self.mechanical_world.integration_parameters.clone();

        self.mechanical_world = mechanical_world;
        self.geometrical_world = geometrical_world;
        self.bodies = bodies;
        self.colliders = colliders;
        self.constraints = joint_constraints;
        self.forces = force_generators;
        self.state
            .action_flags
            .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);
        self.mechanical_world.counters.enable();
        self.geometrical_world
            .maintain(&mut self.bodies, &mut self.colliders);

        #[cfg(feature = "box2d-backend")]
        {
            if self.state.selected_backend == BOX2D_BACKEND {
                self.box2d = Some(Box2dWorld::from_nphysics(
                    &self.mechanical_world,
                    &self.bodies,
                    &self.colliders,
                    &self.constraints,
                    &self.forces,
                ));
            }
        }
    }

    pub fn mechanical_world(&self) -> &DefaultMechanicalWorld<N> {
        &self.mechanical_world
    }

    pub fn mechanical_world_mut(&mut self) -> &mut DefaultMechanicalWorld<N> {
        &mut self.mechanical_world
    }

    #[cfg(feature = "fluids")]
    pub fn set_liquid_world(
        &mut self,
        mut liquid_world: LiquidWorld<N>,
        coupling: ColliderCouplingSet<N, DefaultBodyHandle>,
    ) {
        liquid_world.counters.enable();
        self.fluids = Some(FluidsState {
            world: liquid_world,
            coupling,
        });
    }

    pub fn set_builders(&mut self, builders: Vec<(&'static str, fn(&mut Self))>) {
        self.state.example_names = builders.iter().map(|e| e.0).collect();
        self.builders = builders
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        if !self.camera_locked {
            self.graphics.look_at(at, zoom);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        if !self.camera_locked {
            self.graphics.look_at(eye, at);
        }
    }

    pub fn set_body_color(&mut self, body: DefaultBodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(body, color);
    }

    #[cfg(feature = "fluids")]
    pub fn set_fluid_color(&mut self, fluid: FluidHandle, color: Point3<f32>) {
        self.graphics.set_fluid_color(fluid, color);
    }

    pub fn set_body_wireframe(&mut self, body: DefaultBodyHandle, wireframe_enabled: bool) {
        self.graphics.set_body_wireframe(body, wireframe_enabled);
    }

    pub fn set_collider_color(&mut self, collider: DefaultColliderHandle, color: Point3<f32>) {
        self.graphics.set_collider_color(collider, color);
    }

    #[cfg(feature = "fluids")]
    pub fn set_fluid_rendering_mode(&mut self, mode: FluidRenderingMode) {
        self.graphics.set_fluid_rendering_mode(mode)
    }

    #[cfg(feature = "fluids")]
    pub fn enable_boundary_particles_rendering(&mut self, enabled: bool) {
        self.graphics.enable_boundary_particles_rendering(enabled)
    }

    //    pub fn world(&self) -> &Box<WorldOwner> {
    //        &self.world
    //    }

    pub fn graphics_mut(&mut self) -> &mut GraphicsManager {
        &mut self.graphics
    }

    #[cfg(feature = "dim3")]
    pub fn set_up_axis(&mut self, up_axis: Vector3<f32>) {
        self.graphics.set_up_axis(up_axis);
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
        #[cfg(feature = "fluids")]
        self.callbacks_fluids.clear();
        self.persistant_contacts.clear();
        self.ground_handle = None;
        self.state.grabbed_object = None;
        self.state.grabbed_object_constraint = None;
        self.state.can_grab_behind_ground = false;
        self.graphics.clear(window);
    }

    pub fn add_callback<
        F: FnMut(
                &mut DefaultMechanicalWorld<N>,
                &mut DefaultGeometricalWorld<N>,
                &mut DefaultBodySet<N>,
                &mut DefaultColliderSet<N>,
                &mut GraphicsManager,
                N,
            ) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.callbacks.push(Box::new(callback));
    }

    #[cfg(feature = "fluids")]
    pub fn add_callback_with_fluids<
        F: FnMut(
                &mut LiquidWorld<N>,
                &mut ColliderCouplingSet<N, DefaultBodyHandle>,
                &mut DefaultMechanicalWorld<N>,
                &mut DefaultGeometricalWorld<N>,
                &mut DefaultBodySet<N>,
                &mut DefaultColliderSet<N>,
                &mut GraphicsManager,
                N,
            ) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.callbacks_fluids.push(Box::new(callback));
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
            WindowEvent::Key(Key::R, Action::Release, _) => self
                .state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, true),
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
                let all_groups = &CollisionGroups::new();
                for b in self.geometrical_world.interferences_with_point(
                    &self.colliders,
                    &self.cursor_pos,
                    all_groups,
                ) {
                    if !b.1.query_type().is_proximity_query()
                        && Some(b.1.body()) != self.ground_handle
                    {
                        if let ColliderAnchor::OnBodyPart { body_part, .. } = b.1.anchor() {
                            self.state.grabbed_object = Some(*body_part);
                        } else {
                            continue;
                        }
                    }
                }

                if modifier.contains(Modifiers::Shift) {
                    if let Some(body_part) = self.state.grabbed_object {
                        if Some(body_part.0) != self.ground_handle {
                            self.graphics.remove_body_nodes(window, body_part.0);
                            self.bodies.remove(body_part.0);
                        }
                    }

                    self.state.grabbed_object = None;
                } else if modifier.contains(Modifiers::Alt) {
                    self.state.drawing_ray = Some(self.cursor_pos);
                } else if !modifier.contains(Modifiers::Control) {
                    if let Some(body) = self.state.grabbed_object {
                        if let Some(joint) = self.state.grabbed_object_constraint {
                            let _ = self.constraints.remove(joint);
                        }

                        let body_pos = self
                            .bodies
                            .get(body.0)
                            .unwrap()
                            .part(body.1)
                            .unwrap()
                            .position();
                        let attach1 = self.cursor_pos;
                        let attach2 = body_pos.inverse() * attach1;

                        if let Some(ground) = self.ground_handle {
                            let joint = MouseConstraint::new(
                                BodyPartHandle(ground, 0),
                                body,
                                attach1,
                                attach2,
                                N::one(),
                            );
                            self.state.grabbed_object_constraint =
                                Some(self.constraints.insert(joint));
                        }

                        for node in self.graphics.body_nodes_mut(body.0).unwrap().iter_mut() {
                            node.select()
                        }
                    }

                    event.inhibited = true;
                } else {
                    self.state.grabbed_object = None;
                }
            }
            WindowEvent::MouseButton(MouseButton::Button1, Action::Release, _) => {
                if let Some(body) = self.state.grabbed_object {
                    for n in self.graphics.body_nodes_mut(body.0).unwrap().iter_mut() {
                        n.unselect()
                    }
                }

                if let Some(joint) = self.state.grabbed_object_constraint {
                    let _ = self.constraints.remove(joint);
                }

                if let Some(start) = self.state.drawing_ray {
                    self.graphics.add_ray(Ray::new(
                        na::convert(start),
                        na::convert(self.cursor_pos - start),
                    ));
                }

                self.state.drawing_ray = None;
                self.state.grabbed_object = None;
                self.state.grabbed_object_constraint = None;
            }
            WindowEvent::CursorPos(x, y, modifiers) => {
                self.cursor_pos.x = na::convert(x as f32);
                self.cursor_pos.y = na::convert(y as f32);

                self.cursor_pos = na::convert(
                    self.graphics
                        .camera()
                        .unproject(&na::convert(self.cursor_pos), &na::convert(window.size())),
                );

                let attach2 = self.cursor_pos;
                if self.state.grabbed_object.is_some() {
                    if let Some(constraint) = self
                        .state
                        .grabbed_object_constraint
                        .and_then(|joint| self.constraints.get_mut(joint))
                        .and_then(|joint| {
                            joint.downcast_mut::<MouseConstraint<N, DefaultBodyHandle>>()
                        })
                    {
                        constraint.set_anchor_1(attach2);
                    }
                }

                event.inhibited =
                    modifiers.contains(Modifiers::Control) || modifiers.contains(Modifiers::Shift);
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
                        .unproject(&na::convert(self.cursor_pos), &na::convert(size));
                    let ray = Ray::new(pos, dir);
                    self.graphics.add_ray(ray);

                    event.inhibited = true;
                } else if modifier.contains(Modifiers::Shift) {
                    // XXX: huge and ugly code duplication for the ray cast.
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&na::convert(self.cursor_pos), &na::convert(size));
                    let ray = Ray::new(na::convert(pos), na::convert(dir));

                    // cast the ray
                    let mut mintoi = Bounded::max_value();
                    let mut minb = None;

                    let all_groups = CollisionGroups::new();
                    for (_, b, inter) in self.geometrical_world.interferences_with_ray(
                        &self.colliders,
                        &ray,
                        N::max_value(),
                        &all_groups,
                    ) {
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
                            self.bodies
                                .get_mut(body_part.0)
                                .unwrap()
                                .apply_force_at_point(
                                    body_part.1,
                                    &(ray.dir.normalize() * na::convert::<f32, N>(0.01)),
                                    &ray.point_at(mintoi),
                                    ForceType::Impulse,
                                    true,
                                );
                        }
                    }

                    event.inhibited = true;
                } else if !modifier.contains(Modifiers::Control) {
                    match self.state.grabbed_object {
                        Some(body) => {
                            for n in self.graphics.body_nodes_mut(body.0).unwrap().iter_mut() {
                                n.unselect()
                            }
                        }
                        None => {}
                    }

                    // XXX: huge and uggly code duplication for the ray cast.
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&na::convert(self.cursor_pos), &na::convert(size));
                    let ray = Ray::new(na::convert(pos), na::convert(dir));

                    // cast the ray
                    let mut mintoi = Bounded::max_value();
                    let mut minb = None;

                    let all_groups = CollisionGroups::new();
                    for (_, b, inter) in self.geometrical_world.interferences_with_ray(
                        &self.colliders,
                        &ray,
                        N::max_value(),
                        &all_groups,
                    ) {
                        if ((Some(b.body()) != self.ground_handle)
                            || self.state.can_grab_behind_ground)
                            && !b.query_type().is_proximity_query()
                            && inter.toi < mintoi
                        {
                            mintoi = inter.toi;

                            let subshape = b.shape().subshape_containing_feature(inter.feature);
                            minb = Some(b.body_part(subshape));
                        }
                    }

                    if let Some(body_part_handle) = minb {
                        if self
                            .bodies
                            .get(body_part_handle.0)
                            .unwrap()
                            .status_dependent_ndofs()
                            != 0
                        {
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

                                if let Some(ground_handle) = self.ground_handle {
                                    let constraint = MouseConstraint::new(
                                        BodyPartHandle(ground_handle, 0),
                                        body_part_handle,
                                        attach1,
                                        attach2,
                                        N::one(),
                                    );
                                    self.state.grabbed_object_plane = (attach1, -ray.dir);
                                    self.state.grabbed_object_constraint =
                                        Some(self.constraints.insert(constraint));
                                }

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
                self.cursor_pos.x = na::convert(x as f32);
                self.cursor_pos.y = na::convert(y as f32);

                // update the joint
                if let Some(joint) = self.state.grabbed_object_constraint {
                    let size = window.size();
                    let (pos, dir) = self
                        .graphics
                        .camera()
                        .unproject(&na::convert(self.cursor_pos), &na::convert(size));
                    let (ref ppos, ref pdir) = self.state.grabbed_object_plane;

                    let pos = na::convert(pos);
                    let dir = na::convert(dir);

                    if let Some(inter) = query::ray_toi_with_plane(ppos, pdir, &Ray::new(pos, dir))
                    {
                        let joint = self
                            .constraints
                            .get_mut(joint)
                            .unwrap()
                            .downcast_mut::<MouseConstraint<N, DefaultBodyHandle>>()
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
    Option<&'a mut dyn Camera>,
    Option<&'a mut dyn PlanarCamera>,
    Option<&'a mut dyn PostProcessingEffect>,
);

impl<N: RealField + SupersetOf<f32> + SubsetOf<f32>> State for Testbed<N> {
    fn cameras_and_effect(&mut self) -> CameraEffects<'_> {
        #[cfg(feature = "dim2")]
        let result = (
            None,
            Some(self.graphics.camera_mut() as &mut dyn PlanarCamera),
            None,
        );
        #[cfg(feature = "dim3")]
        let result = (
            Some(self.graphics.camera_mut() as &mut dyn Camera),
            None,
            None,
        );
        result
    }

    fn step(&mut self, window: &mut Window) {
        self.ui
            .update(window, &mut self.mechanical_world, &mut self.state);

        // Handle UI actions.
        {
            let backend_changed = self
                .state
                .action_flags
                .contains(TestbedActionFlags::BACKEND_CHANGED);
            if backend_changed {
                // Marking the example as changed will make the simulation
                // restart with the selected backend.
                self.state
                    .action_flags
                    .set(TestbedActionFlags::BACKEND_CHANGED, false);
                self.state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
                self.camera_locked = true;
            }

            let restarted = self
                .state
                .action_flags
                .contains(TestbedActionFlags::RESTART);
            if restarted {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::RESTART, false);
                self.camera_locked = true;
                self.state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
            }

            let example_changed = self
                .state
                .action_flags
                .contains(TestbedActionFlags::EXAMPLE_CHANGED);
            if example_changed {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, false);
                self.clear(window);
                self.builders[self.state.selected_example].1(self);
                self.camera_locked = false;
            }

            if self
                .state
                .action_flags
                .contains(TestbedActionFlags::RESET_WORLD_GRAPHICS)
            {
                self.state
                    .action_flags
                    .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, false);
                for (handle, _) in self.colliders.iter() {
                    self.graphics.add(window, handle, &self.colliders);
                }

                #[cfg(feature = "fluids")]
                {
                    if let Some(fluids) = &self.fluids {
                        let radius = fluids.world.particle_radius();

                        for (handle, fluid) in fluids.world.fluids().iter() {
                            self.graphics.add_fluid(window, handle, fluid, radius);
                        }

                        for (handle, boundary) in fluids.world.boundaries().iter() {
                            self.graphics.add_boundary(window, handle, boundary, radius);
                        }
                    }
                }
            }

            if example_changed
                || self.state.prev_flags.contains(TestbedStateFlags::WIREFRAME)
                    != self.state.flags.contains(TestbedStateFlags::WIREFRAME)
            {
                self.graphics.toggle_wireframe_mode(
                    &self.colliders,
                    self.state.flags.contains(TestbedStateFlags::WIREFRAME),
                )
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SLEEP)
                != self.state.flags.contains(TestbedStateFlags::SLEEP)
            {
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

            if self
                .state
                .prev_flags
                .contains(TestbedStateFlags::SUB_STEPPING)
                != self.state.flags.contains(TestbedStateFlags::SUB_STEPPING)
            {
                self.mechanical_world
                    .integration_parameters
                    .return_after_ccd_substep =
                    self.state.flags.contains(TestbedStateFlags::SUB_STEPPING);
            }

            if self.state.prev_flags.contains(TestbedStateFlags::SHAPES)
                != self.state.flags.contains(TestbedStateFlags::SHAPES)
            {
                unimplemented!()
            }

            if self.state.prev_flags.contains(TestbedStateFlags::JOINTS)
                != self.state.flags.contains(TestbedStateFlags::JOINTS)
            {
                unimplemented!()
            }

            if example_changed
                || self.state.prev_flags.contains(TestbedStateFlags::AABBS)
                    != self.state.flags.contains(TestbedStateFlags::AABBS)
            {
                if self.state.flags.contains(TestbedStateFlags::AABBS) {
                    self.graphics
                        .show_aabbs(&self.geometrical_world, &self.colliders, window)
                } else {
                    self.graphics.hide_aabbs(window)
                }
            }

            if self
                .state
                .prev_flags
                .contains(TestbedStateFlags::CENTER_OF_MASSES)
                != self
                    .state
                    .flags
                    .contains(TestbedStateFlags::CENTER_OF_MASSES)
            {
                unimplemented!()
            }
        }

        self.state.prev_flags = self.state.flags;

        for event in window.events().iter() {
            let event = self.handle_common_event(event);
            self.handle_special_event(window, event);
        }

        #[cfg(feature = "fluids")]
        let mut fluids_time = 0.0;

        if self.state.running != RunMode::Stop {
            for _ in 0..self.nsteps {
                if self.state.selected_backend == NPHYSICS_BACKEND {
                    self.mechanical_world.step(
                        &mut self.geometrical_world,
                        &mut self.bodies,
                        &mut self.colliders,
                        &mut self.constraints,
                        &mut self.forces,
                    );

                    #[cfg(feature = "fluids")]
                    {
                        fluids_time = instant::now();
                        if let Some(fluids) = &mut self.fluids {
                            let dt = self.mechanical_world.timestep();
                            let gravity = &self.mechanical_world.gravity;
                            fluids.world.step_with_coupling(
                                dt,
                                gravity,
                                &mut fluids
                                    .coupling
                                    .as_manager_mut(&self.colliders, &mut self.bodies),
                            );
                        }

                        fluids_time = instant::now() - fluids_time;
                    }
                }

                #[cfg(feature = "box2d-backend")]
                {
                    if self.state.selected_backend == BOX2D_BACKEND {
                        self.box2d
                            .as_mut()
                            .unwrap()
                            .step(&mut self.mechanical_world);
                        self.box2d
                            .as_mut()
                            .unwrap()
                            .sync(&mut self.bodies, &mut self.colliders);
                    }
                }

                for f in &mut self.callbacks {
                    f(
                        &mut self.mechanical_world,
                        &mut self.geometrical_world,
                        &mut self.bodies,
                        &mut self.colliders,
                        &mut self.graphics,
                        self.time,
                    )
                }

                #[cfg(feature = "fluids")]
                {
                    if let Some(fluid_state) = &mut self.fluids {
                        for f in &mut self.callbacks_fluids {
                            f(
                                &mut fluid_state.world,
                                &mut fluid_state.coupling,
                                &mut self.mechanical_world,
                                &mut self.geometrical_world,
                                &mut self.bodies,
                                &mut self.colliders,
                                &mut self.graphics,
                                self.time,
                            )
                        }
                    }
                }

                if !self.hide_counters {
                    #[cfg(not(feature = "log"))]
                    println!("{}", self.mechanical_world.counters);
                    #[cfg(feature = "log")]
                    debug!("{}", self.mechanical_world.counters);
                }
                self.time += self.mechanical_world.timestep();
            }
        }

        self.graphics
            .draw(&self.geometrical_world, &self.colliders, window);

        #[cfg(feature = "fluids")]
        {
            if let Some(fluids) = &self.fluids {
                self.graphics.draw_fluids(&fluids.world)
            }
        }

        if self.state.flags.contains(TestbedStateFlags::CONTACT_POINTS) {
            draw_collisions(
                window,
                &self.geometrical_world,
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
            let counters = self.mechanical_world.counters;

            #[allow(unused_mut)]
            let mut profile = format!(
                r#"Total: {:.2}ms
Collision detection: {:.2}ms
|_ Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
Island computation: {:.2}ms
Solver: {:.2}ms
|_ Assembly: {:.2}ms
   Velocity resolution: {:.2}ms
   Position resolution: {:.2}ms
CCD: {:.2}ms
|_ # of substeps: {}
   TOI computation: {:.2}ms
   Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
   Solver: {:.2}ms"#,
                counters.step_time(),
                counters.collision_detection_time(),
                counters.broad_phase_time(),
                counters.narrow_phase_time(),
                counters.island_construction_time(),
                counters.solver_time(),
                counters.assembly_time(),
                counters.velocity_resolution_time(),
                counters.position_resolution_time(),
                counters.ccd_time(),
                counters.ccd.num_substeps,
                counters.ccd.toi_computation_time.time(),
                counters.ccd.broad_phase_time.time(),
                counters.ccd.narrow_phase_time.time(),
                counters.ccd.solver_time.time(),
            );

            #[cfg(feature = "fluids")]
            {
                profile = format!(
                    r#"{}
Fluids: {:.2}ms
                        "#,
                    profile, fluids_time,
                )
            }

            if self.state.flags.contains(TestbedStateFlags::PROFILE) {
                window.draw_text(&profile, &Point2::origin(), 45.0, &self.font, &color);
            }
        } else {
            window.draw_text("Paused", &Point2::origin(), 60.0, &self.font, &color);
        }
        //        window.draw_text(CONTROLS, &Point2::new(0.0, 75.0), 40.0, &self.font, &color);
    }
}

fn draw_collisions<N: RealField + SubsetOf<f32>>(
    window: &mut Window,
    geometrical_world: &DefaultGeometricalWorld<N>,
    colliders: &DefaultColliderSet<N>,
    existing: &mut HashMap<ContactId, bool>,
    running: bool,
) {
    for (_, _, _, _, _, manifold) in geometrical_world.contact_pairs(colliders, false) {
        for c in manifold.contacts() {
            existing
                .entry(c.id)
                .and_modify(|value| {
                    if running {
                        *value = true
                    }
                })
                .or_insert(false);

            let color = if c.contact.depth < N::zero() {
                // existing[&c.id] {
                Point3::new(0.0, 0.0, 1.0)
            } else {
                Point3::new(1.0, 0.0, 0.0)
            };

            window.draw_graphics_line(
                &na::convert(c.contact.world1),
                &na::convert(c.contact.world2),
                &color,
            );
        }
    }
}
