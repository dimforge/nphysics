use std::num::{Zero, One};
use nalgebra::traits::inv::Inv;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub, ScalarDiv};
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::{Rotation, Rotate};
use nalgebra::traits::transformation::{Transform, Transformation};
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::indexable::Indexable;
use ncollide::bounding_volume::bounding_volume::HasBoundingVolume;
use ncollide::bounding_volume::aabb::AABB;
use object::volumetric::Volumetric;
use object::implicit_geom::DefaultGeom;
// use constraint::index_proxy::{HasIndexProxy, IndexProxy};

#[deriving(ToStr, Eq, Clone)]
pub enum RigidBodyState {
    Static,
    Dynamic
}

// FIXME: #[deriving(Clone)]
pub struct RigidBody<N, LV, AV, M, II> {
    priv state:          RigidBodyState,
    priv geom:           DefaultGeom<N, LV, M, II>,
    priv local_to_world: M,
    priv world_to_local: M, // FIXME: useless in fact…
    priv lin_vel:        LV,
    priv ang_vel:        AV,
    priv inv_mass:       N,
    priv ls_inv_inertia: II, // NOTE: 'ls' means 'local space'
    priv inv_inertia:    II,
    priv lin_acc:        LV,
    priv ang_acc:        AV,
    priv restitution:    N,
    priv friction:       N,
    priv index:          int
}

impl<N,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV,
     M: One + Translation<LV> + Transform<LV> + Rotate<LV>,
     II: Clone>
RigidBody<N, LV, AV, M, II> {
    fn moved(&mut self, delta: &M) {
        self.geom.transform_by(delta);

        // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
        self.inv_inertia = // FIXME: self.local_to_world.delta_transform() *
            self.ls_inv_inertia.clone() // FIXME:           *
            // FIXME: self.world_to_local.delta_transform()
    }
}

impl<N: Clone, LV, AV, M, II> RigidBody<N, LV, AV, M, II> {
    pub fn geom<'r>(&'r self) -> &'r DefaultGeom<N, LV, M, II> {
        &'r self.geom
    }

    pub fn index(&self) -> int {
        self.index
    }

    pub fn set_index(&mut self, id: int) {
        self.index = id
    }

    pub fn restitution(&self) -> N {
        self.restitution.clone()
    }

    pub fn friction(&self) -> N {
        self.friction.clone()
    }
}

impl<N:   Clone + One + Zero + Div<N, N> + Mul<N, N> + Real + NumCast,
     M:   One + Translation<LV> + Transform<LV> + Rotate<LV>, // FIXME: + DeltaTransform<II>,
     LV:  Clone + Zero + Add<LV, LV> + Neg<LV> + Iterable<N> + Dim,
     AV:  Zero,
     II:  One + Zero + Inv + Mul<II, II> + Indexable<(uint, uint), N> + Clone>
RigidBody<N, LV, AV, M, II> {
    pub fn new(geom:        DefaultGeom<N, LV, M, II>,
               density:     N,
               state:       RigidBodyState,
               restitution: N,
               friction:    N) -> RigidBody<N, LV, AV, M, II> {
        let (inv_mass, inv_inertia) =
            match state {
                Static    => (Zero::zero(), Zero::zero()),
                Dynamic   => {
                    let volume = geom.volume();

                    if volume.is_zero() {
                        fail!("A dynamic body cannot have a zero volume.")
                    }

                    if density.is_zero() {
                        fail!("A dynamic body cannot have a zero density.")
                    }

                    let mass = density * volume;

                    match geom.inertia(&mass).inverse() {
                        Some(ii) => (One::one::<N>() / mass, ii),
                        None     => fail!("A dynamic body cannot have a singular inertia tensor.")
                    }
                },
            };

        let mut res =
            RigidBody {
                state:          state,
                geom:           geom,
                local_to_world: One::one(),
                world_to_local: One::one(),
                lin_vel:        Zero::zero(),
                ang_vel:        Zero::zero(),
                inv_mass:       inv_mass,
                ls_inv_inertia: inv_inertia.clone(),
                inv_inertia:    inv_inertia,
                lin_acc:        Zero::zero(),
                ang_acc:        Zero::zero(),
                friction:       friction,
                restitution:    restitution,
                index:          0
            };

        res.moved(&One::one());

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

impl<N, M, LV, AV, II: Clone> RigidBody<N, LV, AV, M, II> {
    #[inline]
    pub fn inv_inertia(&self) -> II {
        self.inv_inertia.clone()
    }
    #[inline]
    pub fn set_inv_inertia(&mut self, ii: II) {
        self.inv_inertia = ii
    }
}

impl<N,
     M: Clone + Inv + Mul<M, M> + One + Translation<LV> + Transform<LV> + Rotate<LV>,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV,
     II: Mul<II, II> + Clone>
Transformation<M> for RigidBody<N, LV, AV, M, II> {
    #[inline]
    fn transformation(&self) -> M {
        self.local_to_world.clone()
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        self.world_to_local.clone()
    }


    #[inline]
    fn transform_by(&mut self, to_append: &M) {
        self.local_to_world = *to_append * self.local_to_world;

        match self.local_to_world.inverse() {
            Some(l2w) => self.world_to_local = l2w,
            None      => fail!("Internal error: rigid body has a singular local_to_world transform.")
        }
        self.moved(to_append);
    }
}

// FIXME: implement Transfomable too

impl<N,
     M: Translation<LV> + Transform<LV> + Rotate<LV> + One,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV,
     II: Mul<II, II> + Clone>
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

        let mut delta = One::one::<M>();
        delta.translate_by(trans);
        self.moved(&delta);
    }
}

// FIXME: impl<N: Clone,
// FIXME:      M: Clone + Translation<LV> + Transform<LV> + Rotate<LV> + One,
// FIXME:      LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
// FIXME:      AV: Clone,
// FIXME:      II: Clone + Mul<II, II>>
// FIXME: Translatable<LV, RigidBody<N, LV, AV, M, II>> for RigidBody<N, LV, AV, M, II>
// FIXME: {
// FIXME:     #[inline]
// FIXME:     fn translated(&self, trans: &LV) -> RigidBody<N, LV, AV, M, II>
// FIXME:     {
// FIXME:         let mut cpy = self.clone();
// FIXME: 
// FIXME:         cpy.translate_by(trans);
// FIXME: 
// FIXME:         cpy
// FIXME:     }
// FIXME: }

impl<N,
     M: Clone + Translation<LV> + Transform<LV> + Rotate<LV> + Rotation<AV> + One,
     LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
     AV,
     II: Mul<II, II> + Clone>
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

        let mut delta = One::one::<M>();
        delta.rotate_by(rot);
        self.moved(&delta);
    }
}

// FIXME: impl<N:  Clone,
// FIXME:      M:  Clone + Translation<LV> + Transform<LV> + Rotate<LV> + Rotation<AV> + One,
// FIXME:      LV: Clone + Add<LV, LV> + Neg<LV> + Dim,
// FIXME:      AV: Clone,
// FIXME:      II: Clone + Mul<II, II>>
// FIXME: Rotatable<AV, RigidBody<N, LV, AV, M, II>> for RigidBody<N, LV, AV, M, II>
// FIXME: {
// FIXME:     #[inline]
// FIXME:     fn rotated(&self, rot: &AV) -> RigidBody<N, LV, AV, M, II>
// FIXME:     {
// FIXME:         let mut cpy = self.clone();
// FIXME: 
// FIXME:         cpy.rotate_by(rot);
// FIXME: 
// FIXME:         cpy
// FIXME:     }
// FIXME: }

impl<N,
     LV: Bounded + ScalarAdd<N> + ScalarSub<N> + Neg<LV> + Ord + Orderable + ScalarDiv<N> + Clone,
     AV,
     M,
     II>
HasBoundingVolume<AABB<N, LV>> for RigidBody<N, LV, AV, M, II> {
    fn bounding_volume(&self) -> AABB<N, LV> {
        self.geom.aabb()
    }
}
