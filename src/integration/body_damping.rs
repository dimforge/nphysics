use nalgebra::traits::scalar_op::ScalarMul;
use object::body::{Body, RigidBody, SoftBody};
use integration::integrator::Integrator;

pub struct BodyDamping<N, LV, AV, M, II> {
    priv damping_factor: N,
    priv objects:        ~[@mut Body<N, LV, AV, M, II>]
}

impl<N, LV, AV, M, II> BodyDamping<N, LV, AV, M, II> {
    pub fn new(damping_factor: N) -> BodyDamping<N, LV, AV, M, II> {
        BodyDamping {
            damping_factor: damping_factor,
            objects:        ~[]
        }
    }
}

impl<N: Clone, LV: Clone + ScalarMul<N>, AV: Clone + ScalarMul<N>, M: Clone, II: Clone>
Integrator<N, Body<N, LV, AV, M, II>> for BodyDamping<N, LV, AV, M, II> {
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.push(o.clone());
    }

    fn remove(&mut self, _: @mut Body<N, LV, AV, M, II>) {
        fail!("Not yet implemented.");
    }

    fn update(&mut self, _: N) {
        for &o in self.objects.iter() {
            match *o {
                RigidBody(rb) => {
                    let new_lin = rb.lin_vel().scalar_mul(&self.damping_factor);
                    rb.set_lin_vel(new_lin);
                    let new_ang = rb.ang_vel().scalar_mul(&self.damping_factor);
                    rb.set_ang_vel(new_ang);
                },
                SoftBody(_) => {
                    fail!("Not yet implemented.")
                }
            }
        }
    }
}
