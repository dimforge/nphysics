use object::body::{Body, RigidBody, SoftBody};
use integration::integrator::Integrator;

// FIXME: split this on `RigidBodyForceGenerator` and `SoftBodyForceGenerator` ?
pub struct BodyForceGenerator<N, LV, AV, M, II>
{
    priv objects: ~[@mut Body<N, LV, AV, M, II>],
    priv lin_acc: LV,
    priv ang_acc: AV
}

impl<N, LV, AV, M, II> BodyForceGenerator<N, LV, AV, M, II>
{
    pub fn new(lin_acc: LV, ang_acc: AV) -> BodyForceGenerator<N, LV, AV, M, II>
    {
        BodyForceGenerator {
            objects: ~[],
            lin_acc: lin_acc,
            ang_acc: ang_acc
        }
    }
}

impl<N, M, LV: Clone, AV: Clone, II> BodyForceGenerator<N, LV, AV, M, II>
{
    #[inline]
    pub fn lin_acc(&self) -> LV
    { self.lin_acc.clone() }

    pub fn set_lin_acc(&mut self, lin_acc: LV)
    {
        self.lin_acc = lin_acc;

        for o in self.objects.iter()
        { self.write_accs_to(*o) }
    }

    #[inline]
    pub fn ang_acc(&self) -> AV
    { self.ang_acc.clone() }

    pub fn set_ang_acc(&mut self, ang_acc: AV)
    {
        self.ang_acc = ang_acc;

        for o in self.objects.iter()
        { self.write_accs_to(*o) }
    }

    fn write_accs_to(&self, o: &mut Body<N, LV, AV, M, II>)
    {
        match *o
        {
            RigidBody(rb) => {
                rb.set_lin_acc(self.lin_acc.clone());
                rb.set_ang_acc(self.ang_acc.clone());
            },
            SoftBody(sb) => sb.acc = self.lin_acc.clone()
        }
    }
}

impl<N: Clone, M: Clone, LV: Clone, AV: Clone, II: Clone> Integrator<N, Body<N, LV, AV, M, II>>
for BodyForceGenerator<N, LV, AV, M, II>
{
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>)
    {
        self.objects.push(o.clone());

        self.write_accs_to(o)
    }

    fn remove(&mut self, _: @mut Body<N, LV, AV, M, II>)
    {
        fail!("Not yet implemented.");
    }

    fn update(&mut self, _: N)
    { }
}
