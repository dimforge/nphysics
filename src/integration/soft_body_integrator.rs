use nalgebra::traits::scalar_op::ScalarMul;
use object::body::ToSoftBody;
use object::soft_body::SoftBody;
use integration::integrator::Integrator;
use integration::euler;

pub struct SoftBodyExpEulerIntegrator<N, V> {
    priv objects:  ~[@mut SoftBody<N, V>],
}

impl<N: Clone, V: Add<V, V> + ScalarMul<N>, B: ToSoftBody<N, V>> Integrator<N, B>
for SoftBodyExpEulerIntegrator<N, V> {
    fn add(&mut self, o: @mut B) {
        match o.to_soft_body() {
            Some(sb) => self.objects.push(sb),
            None     => { }
        }
    }

    fn remove(&mut self, _: @mut B) {
        // XXX
        fail!("Not yet implemented.");
    }

    fn pre_update(&mut self, dt: N) {
        for o in self.objects.iter() {
            for pt in o.points.mut_iter() {
                let (p, v) = euler::explicit_integrate_wo_rotation(
                    dt.clone(),
                    &pt.position,
                    &pt.velocity,
                    &o.acc
                    );

                pt.position = p;
                pt.velocity = v;
            }
        }
    }

    fn post_update(&mut self, _: N) {
    }
}

pub struct SoftBodySmpEulerIntegrator<N, V> {
    priv objects: ~[@mut SoftBody<N, V>],
}

impl<N: Clone, V: Add<V, V> + ScalarMul<N>, B: ToSoftBody<N, V>> Integrator<N, B>
for SoftBodySmpEulerIntegrator<N, V> {
    fn add(&mut self, o: @mut B) {
        match o.to_soft_body() {
            Some(sb) => self.objects.push(sb),
            None     => { }
        }
    }

    fn remove(&mut self, _: @mut B) {
        // XXX
        fail!("Not yet implemented.");
    }

    fn pre_update(&mut self, dt: N) {
        for o in self.objects.iter() {
            for pt in o.points.mut_iter() {
                let (p, v) = euler::semi_implicit_integrate_wo_rotation(
                    dt.clone(),
                    &pt.position,
                    &pt.velocity,
                    &o.acc
                    );

                pt.position = p;
                pt.velocity = v;
            }
        }
    }

    fn post_update(&mut self, _: N) {
    }
}
