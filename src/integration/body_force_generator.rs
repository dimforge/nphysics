use ncollide::math::{N, LV, AV};
use object::RigidBody;
use integration::Integrator;

pub struct BodyForceGenerator {
    priv lin_acc: LV,
    priv ang_acc: AV
}

impl BodyForceGenerator {
    pub fn new(lin_acc: LV, ang_acc: AV) -> BodyForceGenerator {
        BodyForceGenerator {
            lin_acc: lin_acc,
            ang_acc: ang_acc
        }
    }
}

impl BodyForceGenerator {
    #[inline]
    pub fn lin_acc(&self) -> LV {
        self.lin_acc.clone()
    }

    #[inline]
    pub fn set_lin_acc(&mut self, lin_acc: LV) {
        self.lin_acc = lin_acc;
    }

    #[inline]
    pub fn ang_acc(&self) -> AV {
        self.ang_acc.clone()
    }

    #[inline]
    pub fn set_ang_acc(&mut self, ang_acc: AV) {
        self.ang_acc = ang_acc;
    }
}

impl Integrator<RigidBody> for BodyForceGenerator {
    #[inline]
    fn update(&mut self, _: N, rb: &mut RigidBody) {
        rb.set_lin_acc(self.lin_acc.clone());
        rb.set_ang_acc(self.ang_acc.clone());
    }
}
