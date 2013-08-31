use std::num::{Zero, One};
use nalgebra::mat::{Translation, Rotation, Rotate, Transformation, Transform, Inv, Indexable};
use nalgebra::vec::{VecExt, AlgebraicVecExt, Dim};
use ncollide::bounding_volume::bounding_volume::HasBoundingVolume;
use ncollide::bounding_volume::aabb::{AABB, HasAABB};
use object::volumetric::{InertiaTensor, Volumetric};
use object::implicit_geom::DefaultGeom;
// use constraint::index_proxy::{HasIndexProxy, IndexProxy};

#[deriving(ToStr, Eq, Clone)]
pub enum RigidBodyState {
    Static,
    Dynamic
}

// FIXME: #[deriving(Clone)]
pub struct RigidBody<N, LV, AV, M, II> {
    priv state:                RigidBodyState,
    priv geom:                 DefaultGeom<N, LV, M, II>,
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

impl<N, LV, AV, M: Transform<LV>, II: InertiaTensor<N, LV, M>>
RigidBody<N, LV, AV, M, II> {
    fn update_inertia_tensor(&mut self) {
        // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
        self.inv_inertia    = self.ls_inv_inertia.to_world_space(&self.local_to_world);
    }
}

impl<N, LV: ToStr, AV, M: Transform<LV>, II>
RigidBody<N, LV, AV, M, II> {
    fn update_center_of_mass(&mut self) {
        self.center_of_mass = self.local_to_world.transform(&self.ls_center_of_mass);
    }
}

impl<N: Clone, LV, AV, M, II> RigidBody<N, LV, AV, M, II> {
    pub fn transform_ref<'r>(&'r self) -> &'r M {
        &'r self.local_to_world
    }

    pub fn geom<'r>(&'r self) -> &'r DefaultGeom<N, LV, M, II> {
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

impl<N:   Clone + One + Zero + Div<N, N> + Mul<N, N> + Real + NumCast + ToStr,
     M:   One + Translation<LV> + Transform<LV> + Rotate<LV>, // FIXME: + DeltaTransform<II>,
     LV:  Clone + VecExt<N> + ToStr,
     AV:  Zero,
     II:  One + Zero + Inv + Mul<II, II> + Indexable<(uint, uint), N> + InertiaTensor<N, LV, M> +
          Add<II, II> + Dim + Clone + ToStr>
RigidBody<N, LV, AV, M, II> {
    pub fn new(geom:        DefaultGeom<N, LV, M, II>,
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

                    let (m, c, ii) = geom.mass_properties(&density);

                    if m.is_zero() {
                        fail!("A dynamic body must not have a zero volume.")
                    }

                    let ii_wrt_com = 
                        ii.to_relative_wrt_point(&m, &c)
                          .inverse()
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
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim + ToStr,
     AV,
     II: Mul<II, II> + Inv + InertiaTensor<N, LV, M> + Clone>
Transformation<M> for RigidBody<N, LV, AV, M, II> {
    #[inline]
    fn transformation(&self) -> M {
        self.local_to_world.clone()
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        self.local_to_world.inverse().unwrap()
    }

    #[inline]
    fn transform_by(&mut self, to_append: &M) {
        self.local_to_world = *to_append * self.local_to_world;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn transformed(&self, _: &M) -> RigidBody<N, LV, AV, M, II> {
        fail!("`transformed` is not yet implemented for RigidBodies.")
    }
}

// FIXME: implement Transfomable too

impl<N,
     M: Translation<LV> + Transform<LV> + Rotate<LV> + One,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim + ToStr,
     AV,
     II>
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
    fn translate_by(&mut self, trans: &LV) {
        self.local_to_world.translate_by(trans);
        self.update_center_of_mass();
    }

    #[inline]
    fn translated(&self, _: &LV) -> RigidBody<N, LV, AV, M, II> {
        fail!("`translated` is not yet implemented for RigidBodies.")
    }
}

impl<N,
     M: Clone + Translation<LV> + Transform<LV> + Rotate<LV> + Rotation<AV> + One,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim + ToStr,
     AV,
     II: Mul<II, II> + InertiaTensor<N, LV, M> + Clone>
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
    fn rotated(&self, _: &AV) -> RigidBody<N, LV, AV, M, II> {
        fail!("`rotated` is not yet implemented for RigidBodies.")
    }
}

impl<N:  NumCast + Primitive + Orderable + Algebraic + Signed + Clone + ToStr,
     LV: AlgebraicVecExt<N> + Clone + ToStr,
     AV,
     M: Translation<LV> + Rotate<LV> + Transform<LV> + Mul<M, M>,
     II>
HasBoundingVolume<AABB<N, LV>> for RigidBody<N, LV, AV, M, II> {
    fn bounding_volume(&self) -> AABB<N, LV> {
        self.geom.aabb(&self.local_to_world)
    }
}
