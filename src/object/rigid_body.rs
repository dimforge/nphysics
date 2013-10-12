use std::num::{Zero, One};
use nalgebra::na::{
    Translation, Rotation, Rotate, AbsoluteRotate,
    Transformation, Transform,
    Cast, Inv, Indexable,
    VecExt, AlgebraicVecExt, Dim
};
use ncollide::bounding_volume::{HasBoundingVolume, AABB, HasAABB};
use ncollide::geom::Geom;
use object::volumetric::{InertiaTensor, Volumetric};
// use constraint::index_proxy::{HasIndexProxy, IndexProxy};

#[deriving(ToStr, Eq, Clone, Encodable, Decodable)]
pub enum RigidBodyState {
    Static,
    Dynamic
}

#[deriving(Clone, Encodable, Decodable)]
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


impl<N, LV: Zero, AV: Zero, M, II> RigidBody<N, LV, AV, M, II> {
    pub fn deactivate(&mut self) {
        self.lin_vel = Zero::zero();
        self.ang_vel = Zero::zero();
        self.active  = false;
    }
}

impl<N, LV, AV, M: Transform<LV>, II: InertiaTensor<N, LV, AV, M>>
RigidBody<N, LV, AV, M, II> {
    fn update_inertia_tensor(&mut self) {
        // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
        self.inv_inertia    = self.ls_inv_inertia.to_world_space(&self.local_to_world);
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

impl<N:   Clone + One + Zero + Div<N, N> + Mul<N, N> + Real + Cast<f32>,
     M:   One + Translation<LV> + Transform<LV> + Rotate<LV>,
     LV:  Clone + VecExt<N>,
     AV:  Zero,
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
                Static    => (Zero::zero(), Zero::zero(), Zero::zero()),
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
                          i_wrt_com.inverted()
                          .expect("A dynamic body must not have a singular inertia tensor.");

                    let _1: N = One::one();
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
                local_to_world:       One::one(),
                lin_vel:              Zero::zero(),
                ang_vel:              Zero::zero(),
                inv_mass:             inv_mass,
                ls_inv_inertia:       inv_inertia.clone(),
                inv_inertia:          inv_inertia,
                ls_center_of_mass:    center_of_mass,
                center_of_mass:       Zero::zero(),
                lin_acc:              Zero::zero(),
                ang_acc:              Zero::zero(),
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

impl<N:  Clone,
     M:  Clone + Inv + Mul<M, M> + One + Translation<LV> + Transform<LV> + Rotate<LV>,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV: Clone,
     II: Mul<II, II> + Inv + InertiaTensor<N, LV, AV, M> + Clone>
Transformation<M> for RigidBody<N, LV, AV, M, II> {
    #[inline]
    fn transformation(&self) -> M {
        self.local_to_world.clone()
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        self.local_to_world.inverted().unwrap()
    }

    #[inline]
    fn transform_by(&mut self, to_append: &M) {
        self.local_to_world = *to_append * self.local_to_world;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn transformed(&self, m: &M) -> RigidBody<N, LV, AV, M, II> {
        let mut res = self.clone();

        res.transform_by(m);

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

impl<N:  Clone,
     M:  Clone + Translation<LV> + Transform<LV> + Rotate<LV> + One,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV: Clone,
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
    fn translate_by(&mut self, t: &LV) {
        self.local_to_world.translate_by(t);
        self.update_center_of_mass();
    }

    #[inline]
    fn translated(&self, t: &LV) -> RigidBody<N, LV, AV, M, II> {
        let mut res = self.clone();

        res.translate_by(t);

        res
    }

    #[inline]
    fn set_translation(&mut self, t: LV) {
        self.local_to_world.set_translation(t);

        self.update_center_of_mass();
    }
}

impl<N:  Clone,
     M:  Clone + Translation<LV> + Transform<LV> + Rotate<LV> + Rotation<AV> + One,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV: Clone,
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
    fn rotate_by(&mut self, rot: &AV) {
        self.local_to_world.rotate_by(rot);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn rotated(&self, rot: &AV) -> RigidBody<N, LV, AV, M, II> {
        let mut res = self.clone();

        res.rotate_by(rot);

        res
    }

    #[inline]
    fn set_rotation(&mut self, r: AV) {
        self.local_to_world.set_rotation(r);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }
}

impl<N:  Cast<f32> + Primitive + Orderable + Algebraic + Signed + Clone,
     LV: AlgebraicVecExt<N> + Clone,
     AV,
     M: Translation<LV> + Rotate<LV> + Transform<LV> + AbsoluteRotate<LV> + Mul<M, M>,
     II>
HasBoundingVolume<AABB<N, LV>> for RigidBody<N, LV, AV, M, II> {
    fn bounding_volume(&self) -> AABB<N, LV> {
        self.geom.aabb(&self.local_to_world)
    }
}
