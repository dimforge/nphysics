use std::num::{Zero, One};
use nalgebra::na::{
    Translation, Rotation, Rotate, AbsoluteRotate,
    Transformation, Transform,
    Cast, Inv, Indexable,
    VecExt, AlgebraicVecExt, Dim
};
use nalgebra::na;
use ncollide::bounding_volume::{HasBoundingVolume, AABB, HasAABB};
use ncollide::geom::Geom;
use object::volumetric::{InertiaTensor, Volumetric};
// use constraint::index_proxy::{HasIndexProxy, IndexProxy};

#[deriving(ToStr, Eq, Clone, Encodable, Decodable)]
pub enum RigidBodyState {
    Static,
    Dynamic
}

pub struct RigidBody<N, LV, AV, M, II> {
    priv state:                RigidBodyState,
    priv geom:                 Geom<N, LV, M>,
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
    priv active:               bool
}

impl<N:  Send + Freeze + Clone,
     LV: Send + Freeze + Clone,
     AV: Clone,
     M:  Send + Freeze + Clone,
     II: Clone>
Clone for RigidBody<N, LV, AV, M, II> {
    fn clone(&self) -> RigidBody<N, LV, AV, M, II> {
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
            active:            self.active.clone()
        }
    }
}


impl<N, LV: Zero, AV: Zero, M, II> RigidBody<N, LV, AV, M, II> {
    pub fn deactivate(&mut self) {
        self.lin_vel = na::zero();
        self.ang_vel = na::zero();
        self.active  = false;
    }
}

impl<N, LV, AV, M: Transform<LV>, II: InertiaTensor<N, LV, AV, M>>
RigidBody<N, LV, AV, M, II> {
    fn update_inertia_tensor(&mut self) {
        // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
        self.inv_inertia = self.ls_inv_inertia.to_world_space(&self.local_to_world);
    }
}

impl<N, LV, AV, M: Transform<LV>, II>
RigidBody<N, LV, AV, M, II> {
    fn update_center_of_mass(&mut self) {
        self.center_of_mass = self.local_to_world.transform(&self.ls_center_of_mass);
    }
}

impl<N: Clone, LV, AV, M, II> RigidBody<N, LV, AV, M, II> {
    pub fn transform_ref<'r>(&'r self) -> &'r M {
        &'r self.local_to_world
    }

    pub fn geom<'r>(&'r self) -> &'r Geom<N, LV, M> {
        &'r self.geom
    }

    pub fn index(&self) -> int {
        self.index
    }

    pub fn set_index(&mut self, id: int) {
        self.index = id
    }

    pub fn center_of_mass<'r>(&'r self) -> &'r LV {
        &'r self.center_of_mass
    }

    pub fn restitution(&self) -> N {
        self.restitution.clone()
    }

    pub fn friction(&self) -> N {
        self.friction.clone()
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn activate(&mut self) {
        self.active = true;
    }
}

impl<N:   Send + Freeze + Clone + One + Zero + Div<N, N> + Mul<N, N> + Real + Cast<f32>,
     LV:  Send + Freeze + Clone + VecExt<N>,
     AV:  Zero,
     M:   Send + Freeze + One + Translation<LV> + Transform<LV> + Rotate<LV>,
     II:  One + Zero + Inv + Mul<II, II> + Indexable<(uint, uint), N> + InertiaTensor<N, LV, AV, M> +
          Add<II, II> + Dim + Clone>
RigidBody<N, LV, AV, M, II> {
    pub fn new(geom:        Geom<N, LV, M>,
               density:     N,
               state:       RigidBodyState,
               restitution: N,
               friction:    N) -> RigidBody<N, LV, AV, M, II> {
        let (inv_mass, center_of_mass, inv_inertia) =
            match state {
                Static    => (na::zero(), na::zero(), na::zero()),
                Dynamic   => {
                    if density.is_zero() {
                        fail!("A dynamic body must not have a zero density.")
                    }

                    let mprops: (N, LV, II) = geom.mass_properties(&density);
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
                        ii_wrt_com
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
                active:               true
            };

        res.update_center_of_mass();
        res.update_inertia_tensor();

        res
    }
}

impl<N, LV, AV, M, II> RigidBody<N, LV, AV, M, II> {
    #[inline]
    pub fn can_move(&self) -> bool {
        match self.state {
            Dynamic => true,
            _       => false
        }
    }
}

impl<N, M, LV: Clone, AV, II> RigidBody<N, LV, AV, M, II> {
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
}

impl<N, M, LV, AV: Clone, II> RigidBody<N, LV, AV, M, II> {
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
}

impl<N: Clone, M, LV, AV, II> RigidBody<N, LV, AV, M, II> {
    #[inline]
    pub fn inv_mass(&self) -> N {
        self.inv_mass.clone()
    }
    #[inline]
    pub fn set_inv_mass(&mut self, m: N) {
        self.inv_mass = m
    }
}

impl<N, M, LV, AV, II> RigidBody<N, LV, AV, M, II> {
    #[inline]
    pub fn inv_inertia<'r>(&'r self) -> &'r II {
        &'r self.inv_inertia
    }

    #[inline]
    pub fn set_inv_inertia(&mut self, ii: II) {
        self.inv_inertia = ii
    }
}

impl<N:  Send + Freeze + Clone,
     LV: Send + Freeze + Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV: Clone,
     M:  Send + Freeze + Clone + Inv + Mul<M, M> + One + Translation<LV> + Transform<LV> +
         Transformation<M> + Rotate<LV>,
     II: Mul<II, II> + Inv + InertiaTensor<N, LV, AV, M> + Clone>
Transformation<M> for RigidBody<N, LV, AV, M, II> {
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
    fn append_transformation_cpy(rb: &RigidBody<N, LV, AV, M, II>, m: &M) -> RigidBody<N, LV, AV, M, II> {
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
    fn prepend_transformation_cpy(rb: &RigidBody<N, LV, AV, M, II>, m: &M) -> RigidBody<N, LV, AV, M, II> {
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

impl<N:  Send + Freeze + Clone,
     LV: Send + Freeze + Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV: Clone,
     M:  Send + Freeze + Clone + Translation<LV> + Transform<LV> + Rotate<LV> + One,
     II: Clone>
Translation<LV> for RigidBody<N, LV, AV, M, II> {
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
    fn append_translation_cpy(rb: &RigidBody<N, LV, AV, M, II>, t: &LV) -> RigidBody<N, LV, AV, M, II> {
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
    fn prepend_translation_cpy(rb: &RigidBody<N, LV, AV, M, II>, t: &LV) -> RigidBody<N, LV, AV, M, II> {
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

impl<N:  Clone + Send + Freeze,
     LV: Clone + Send + Freeze + Add<LV, LV> + Neg<LV> + Dim,
     AV: Clone,
     M:  Clone + Send + Freeze + Translation<LV> + Transform<LV> + Rotate<LV> + Rotation<AV> + One,
     II: Mul<II, II> + InertiaTensor<N, LV, AV, M> + Clone>
Rotation<AV> for RigidBody<N, LV, AV, M, II> {
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
    fn append_rotation_cpy(rb: &RigidBody<N, LV, AV, M, II>, rot: &AV) -> RigidBody<N, LV, AV, M, II> {
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
    fn prepend_rotation_cpy(rb: &RigidBody<N, LV, AV, M, II>, rot: &AV) -> RigidBody<N, LV, AV, M, II> {
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

impl<N:  Send + Freeze + Cast<f32> + Primitive + Orderable + Algebraic + Signed + Clone,
     LV: Send + Freeze + AlgebraicVecExt<N> + Clone,
     AV,
     M:  Send + Freeze + Translation<LV> + Rotate<LV> + Transform<LV> + AbsoluteRotate<LV> +
         Mul<M, M>,
     II>
HasBoundingVolume<AABB<N, LV>> for RigidBody<N, LV, AV, M, II> {
    fn bounding_volume(&self) -> AABB<N, LV> {
        self.geom.aabb(&self.local_to_world)
    }
}
