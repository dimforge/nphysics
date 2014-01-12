use std::num::Zero;
use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Transformation, Translation, Rotation};
use nalgebra::na;
use nalgebra::na::Transform;
use ncollide::bounding_volume::{HasBoundingVolume, AABB, HasAABB};
use ncollide::geom::Geom;
use ncollide::volumetric::{InertiaTensor, Volumetric};
use ncollide::math::{N, LV, AV, M, II};

#[deriving(ToStr, Eq, Clone, Encodable, Decodable)]
pub enum RigidBodyState {
    Static,
    Dynamic
}

#[deriving(ToStr, Eq, Clone, Encodable, Decodable)]
pub enum ActivationState {
    Active(N),
    Inactive,
    Deleted
}

impl ActivationState {
    pub fn energy(&self) -> N {
        match *self {
            Active(n) => n.clone(),
            Inactive  => na::zero(),
            Deleted   => na::zero()
        }
    }
}

pub struct RigidBody {
    priv state:                RigidBodyState,
    priv geom:                 Rc<~Geom>,
    priv local_to_world:       M,
    priv lin_vel:              LV,
    priv ang_vel:              AV,
    priv inv_mass:             N,
    priv ls_inv_inertia:       II,
    priv inv_inertia:          II,
    priv ls_center_of_mass:    LV,
    priv center_of_mass:       LV,
    priv lin_acc:              LV,
    priv ang_acc:              AV,
    priv restitution:          N,
    priv friction:             N,
    priv index:                int,
    priv activation_state:     ActivationState
}

impl Clone for RigidBody {
    fn clone(&self) -> RigidBody {
        RigidBody {
            state:             self.state.clone(),
            geom:              self.geom.clone(),
            local_to_world:    self.local_to_world.clone(),
            lin_vel:           self.lin_vel.clone(),
            ang_vel:           self.ang_vel.clone(),
            inv_mass:          self.inv_mass.clone(),
            ls_inv_inertia:    self.ls_inv_inertia.clone(),
            inv_inertia:       self.inv_inertia.clone(),
            ls_center_of_mass: self.ls_center_of_mass.clone(),
            center_of_mass:    self.center_of_mass.clone(),
            lin_acc:           self.lin_acc.clone(),
            ang_acc:           self.ang_acc.clone(),
            restitution:       self.restitution.clone(),
            friction:          self.friction.clone(),
            index:             self.index.clone(),
            activation_state:  self.activation_state.clone()
        }
    }
}


impl RigidBody {
    #[inline]
    pub fn deactivate(&mut self) {
        self.lin_vel          = na::zero();
        self.ang_vel          = na::zero();
        self.activation_state = Inactive;
    }

    #[inline]
    pub fn delete(&mut self) {
        self.activation_state = Deleted;
    }

    #[inline]
    fn update_inertia_tensor(&mut self) {
        // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
        self.inv_inertia = self.ls_inv_inertia.to_world_space(&self.local_to_world);
    }

    #[inline]
    fn update_center_of_mass(&mut self) {
        self.center_of_mass = self.local_to_world.transform(&self.ls_center_of_mass);
    }

    #[inline]
    pub fn transform_ref<'r>(&'r self) -> &'r M {
        &'r self.local_to_world
    }

    #[inline]
    pub fn geom<'r>(&'r self) -> &'r Geom {
        let res: &'r Geom = *self.geom.borrow();

        res
    }

    #[inline]
    pub fn index(&self) -> int {
        self.index
    }

    #[inline]
    pub fn set_index(&mut self, id: int) {
        self.index = id
    }

    #[inline]
    pub fn center_of_mass<'r>(&'r self) -> &'r LV {
        &'r self.center_of_mass
    }

    #[inline]
    pub fn restitution(&self) -> N {
        self.restitution.clone()
    }

    #[inline]
    pub fn friction(&self) -> N {
        self.friction.clone()
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        self.activation_state != Inactive
    }

    #[inline]
    pub fn activation_state<'a>(&'a self) -> &'a ActivationState {
        &'a self.activation_state
    }

    #[inline]
    pub fn activate(&mut self, energy: N) {
        self.activation_state = Active(energy);
    }

    pub fn new<G: 'static + Send + Geom>(geom:        G,
                                                       density:     N,
                                                       state:       RigidBodyState,
                                                       restitution: N,
                                                       friction:    N)
                                                       -> RigidBody {
        RigidBody::new_with_shared_geom(Rc::new(~geom as ~Geom),
                                        density,
                                        state,
                                        restitution,
                                        friction)
    }

    pub fn new_with_shared_geom(geom:        Rc<~Geom>,
                                density:     N,
                                state:       RigidBodyState,
                                restitution: N,
                                friction:    N)
                                -> RigidBody {
        let (inv_mass, center_of_mass, inv_inertia, active) =
            match state {
                Static    => (na::zero(), na::zero(), na::zero(), Inactive),
                Dynamic   => {
                    if density.is_zero() {
                        fail!("A dynamic body must not have a zero density.")
                    }

                    let mprops: (N, LV, II) = geom.borrow().mass_properties(&density);
                    let (m, c, ii) = mprops;

                    if m.is_zero() {
                        fail!("A dynamic body must not have a zero volume.")
                    }

                    let i_wrt_com: II = ii.to_relative_wrt_point(&m, &c);
                    let ii_wrt_com: II =
                          na::inv(&i_wrt_com)
                          .expect("A dynamic body must not have a singular inertia tensor.");

                    let _1: N = na::one();

                    (
                        _1 / m,
                        c,
                        ii_wrt_com,
                        Active(Bounded::max_value())
                    )
                },
            };

        let mut res =
            RigidBody {
                state:                state,
                geom:                 geom,
                local_to_world:       na::one(),
                lin_vel:              na::zero(),
                ang_vel:              na::zero(),
                inv_mass:             inv_mass,
                ls_inv_inertia:       inv_inertia.clone(),
                inv_inertia:          inv_inertia,
                ls_center_of_mass:    center_of_mass,
                center_of_mass:       na::zero(),
                lin_acc:              na::zero(),
                ang_acc:              na::zero(),
                friction:             friction,
                restitution:          restitution,
                index:                0,
                activation_state:     active
            };

        res.update_center_of_mass();
        res.update_inertia_tensor();

        res
    }

    #[inline]
    pub fn can_move(&self) -> bool {
        match self.state {
            Dynamic => true,
            _       => false
        }
    }

    #[inline]
    pub fn lin_vel(&self) -> LV {
        self.lin_vel.clone()
    }

    #[inline]
    pub fn set_lin_vel(&mut self, lv: LV) {
        self.lin_vel = lv
    }

    #[inline]
    pub fn lin_acc(&self) -> LV {
        self.lin_acc.clone()
    }

    #[inline]
    pub fn set_lin_acc(&mut self, lf: LV) {
        self.lin_acc = lf
    }

    #[inline]
    pub fn ang_vel(&self) -> AV {
        self.ang_vel.clone()
    }
    #[inline]
    pub fn set_ang_vel(&mut self, av: AV) {
        self.ang_vel = av
    }

    #[inline]
    pub fn ang_acc(&self) -> AV {
        self.ang_acc.clone()
    }
    #[inline]
    pub fn set_ang_acc(&mut self, af: AV) {
        self.ang_acc = af
    }

    #[inline]
    pub fn inv_mass(&self) -> N {
        self.inv_mass.clone()
    }
    #[inline]
    pub fn set_inv_mass(&mut self, m: N) {
        self.inv_mass = m
    }

    #[inline]
    pub fn inv_inertia<'r>(&'r self) -> &'r II {
        &'r self.inv_inertia
    }

    #[inline]
    pub fn set_inv_inertia(&mut self, ii: II) {
        self.inv_inertia = ii
    }
}

impl Transformation<M> for RigidBody {
    #[inline]
    fn transformation(&self) -> M {
        self.local_to_world.clone()
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        na::inv(&self.local_to_world).unwrap()
    }

    #[inline]
    fn append_transformation(&mut self, to_append: &M) {
        self.local_to_world.append_transformation(to_append);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn append_transformation_cpy(rb: &RigidBody, m: &M) -> RigidBody {
        let mut res = rb.clone();

        res.append_transformation(m);

        res
    }

    #[inline]
    fn prepend_transformation(&mut self, to_prepend: &M) {
        self.local_to_world.prepend_transformation(to_prepend);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn prepend_transformation_cpy(rb: &RigidBody, m: &M) -> RigidBody {
        let mut res = rb.clone();

        res.prepend_transformation(m);

        res
    }

    #[inline]
    fn set_transformation(&mut self, m: M) {
        self.local_to_world = m;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }
}

// FIXME: implement Transfomable too

impl Translation<LV> for RigidBody {
    #[inline]
    fn translation(&self) -> LV {
        self.local_to_world.translation()
    }

    #[inline]
    fn inv_translation(&self) -> LV {
        self.local_to_world.inv_translation()
    }

    #[inline]
    fn append_translation(&mut self, t: &LV) {
        self.local_to_world.append_translation(t);
        self.update_center_of_mass();
    }

    #[inline]
    fn append_translation_cpy(rb: &RigidBody, t: &LV) -> RigidBody {
        let mut res = rb.clone();

        res.append_translation(t);

        res
    }

    #[inline]
    fn prepend_translation(&mut self, t: &LV) {
        self.local_to_world.prepend_translation(t);
        self.update_center_of_mass();
    }

    #[inline]
    fn prepend_translation_cpy(rb: &RigidBody, t: &LV) -> RigidBody {
        let mut res = rb.clone();

        res.prepend_translation(t);

        res
    }

    #[inline]
    fn set_translation(&mut self, t: LV) {
        self.local_to_world.set_translation(t);

        self.update_center_of_mass();
    }
}

impl Rotation<AV> for RigidBody {
    #[inline]
    fn rotation(&self) -> AV {
        self.local_to_world.rotation()
    }

    #[inline]
    fn inv_rotation(&self) -> AV {
        self.local_to_world.inv_rotation()
    }

    #[inline]
    fn append_rotation(&mut self, rot: &AV) {
        self.local_to_world.append_rotation(rot);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn append_rotation_cpy(rb: &RigidBody, rot: &AV) -> RigidBody {
        let mut res = rb.clone();

        res.append_rotation(rot);

        res
    }

    #[inline]
    fn prepend_rotation(&mut self, rot: &AV) {
        self.local_to_world.prepend_rotation(rot);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn prepend_rotation_cpy(rb: &RigidBody, rot: &AV) -> RigidBody {
        let mut res = rb.clone();

        res.prepend_rotation(rot);

        res
    }

    #[inline]
    fn set_rotation(&mut self, r: AV) {
        self.local_to_world.set_rotation(r);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }
}

impl HasBoundingVolume<AABB> for RigidBody {
    fn bounding_volume(&self) -> AABB {
        self.geom.borrow().aabb(&self.local_to_world)
    }
}

impl HasBoundingVolume<AABB> for Rc<RefCell<RigidBody>> {
    fn bounding_volume(&self) -> AABB {
        let bself = self.borrow().borrow();
        bself.get().geom().aabb(&bself.get().local_to_world)
    }
}
