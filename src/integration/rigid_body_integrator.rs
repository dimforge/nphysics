use std::num::One;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::transformation::{Transformation, Transform};
use nalgebra::traits::rotation::{Rotation, Rotate};
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::inv::Inv;
use object::body::ToRigidBody;
use object::rigid_body::RigidBody;
use object::volumetric::InertiaTensor;
use integration::integrator::Integrator;
use integration::euler;

pub struct RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    priv objects: ~[@mut RigidBody<N, LV, AV, M, II>],
}

impl<N, LV, AV, M, II> RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    pub fn new() -> RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
        RigidBodyExpEulerIntegrator {
            objects: ~[]
        }
    }
}

impl<N:  Clone,
     M:  Clone + Inv + Mul<M, M> + Rotation<AV> + Rotate<LV> + Translation<LV> +
     Translatable<LV, M> + Transform<LV> + One,
     LV: Clone + Add<LV, LV> + ScalarMul<N> + Neg<LV> + Dim,
     AV: Clone + Add<AV, AV> + ScalarMul<N>,
     II: Clone + Mul<II, II> + InertiaTensor<M> + Inv,
     B: ToRigidBody<N, LV, AV, M, II>>
Integrator<N, B> for RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    fn add(&mut self, o: @mut B) {
        match o.to_rigid_body() {
            Some(rb) => self.objects.push(rb),
            None     => { }
        }
    }

    fn remove(&mut self, _: @mut B) {
        // XXX
        fail!("Not yet implemented.");
    }

    fn pre_update(&mut self, dt: N) {
        for o in self.objects.iter() {
            if o.can_move() {
                let (t, lv, av) = euler::explicit_integrate(
                    dt.clone(),
                    o.transform_ref(),
                    &o.lin_vel(),
                    &o.ang_vel(),
                    &o.lin_acc(),
                    &o.ang_acc()
                    );

                o.transform_by(&t);
                o.set_lin_vel(lv);
                o.set_ang_vel(av);
            }
        }
    }

    fn post_update(&mut self, _: N) {
    }
}

pub struct RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    priv objects: ~[@mut RigidBody<N, LV, AV, M, II>],
}

impl<N, LV, AV, M, II> RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    pub fn new() -> RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
        RigidBodySmpEulerIntegrator {
            objects: ~[]
        }
    }
}

impl<N:  Clone,
     M:  Clone + Inv + Mul<M, M> + Rotation<AV> + Transform<LV> +
     Translation<LV> + Translatable<LV, M> + Rotate<LV> + One,
     LV: Clone + Add<LV, LV> + ScalarMul<N> + Neg<LV> + Dim,
     AV: Clone + Add<AV, AV> + ScalarMul<N>,
     II: Clone + Mul<II, II> + Inv + InertiaTensor<M>,
     B: ToRigidBody<N, LV, AV, M, II>>
Integrator<N, B>
for RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    fn add(&mut self, o: @mut B) {
        match o.to_rigid_body() {
            Some(rb) => self.objects.push(rb),
            None     => { }
        }
    }

    fn remove(&mut self, _: @mut B) {
        // XXX
        fail!("Not yet implemented.");
    }

    fn pre_update(&mut self, _: N) {
    }

    fn post_update(&mut self, dt: N) {
        for o in self.objects.iter() {
            if o.can_move() {
                // o.restore_transform();

                let (t, lv, av) = euler::semi_implicit_integrate(
                    dt.clone(),
                    o.transform_ref(),
                    &o.lin_vel(),
                    &o.ang_vel(),
                    &o.lin_acc(),
                    &o.ang_acc());

                o.transform_by(&t);
                o.set_lin_vel(lv);
                o.set_ang_vel(av);
            }
        }
    }
}
