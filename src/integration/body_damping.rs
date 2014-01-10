use ncollide::math::N;
use integration::Integrator;
use object::RigidBody;

pub struct BodyDamping {
    priv linear_damping:  N,
    priv angular_damping: N
}

impl BodyDamping {
    #[inline]
    pub fn new(linear_damping: N, angular_damping: N) -> BodyDamping {
        BodyDamping {
            linear_damping:  linear_damping,
            angular_damping: angular_damping,
        }
    }
}

impl Integrator<RigidBody> for BodyDamping {
    fn update(&mut self, _: N, rb: &mut RigidBody) {
        let new_lin = rb.lin_vel() * self.linear_damping;

        rb.set_lin_vel(new_lin);
        let new_ang = rb.ang_vel() * self.angular_damping;
        rb.set_ang_vel(new_ang);
    }
}
