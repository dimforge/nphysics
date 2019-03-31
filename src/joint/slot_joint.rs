use na::{self, RealField, U3, VectorSlice3, Vector, Point, Matrix3, Isometry, Translation3, UnitQuaternion};

use crate::utils::GeneralizedCross;
use crate::joint::Joint;
use crate::solver::IntegrationParameters;
use crate::math::{Velocity, JacobianSliceMut};


#[derive(Copy, Clone, Debug)]
pub struct SlotJoint<N: RealField> {
    shift: Vector<N>,
    axis:  Unit<Vector<N>>,

    pos: N,
    rot: Rotation<N>,

    jacobian_v:     Matrix3<N>,
    jacobian_dot_v: Matrix3<N>,
}

impl<N: RealField> SlotJoint<N> {
    pub fn new(center: Point<N>, axisangle: Vector<N>) -> Self {
        SlotJoint {
            shift:          -center.coords,
            rot:            UnitQuaternion::new(axisangle),
            jacobian_v:     na::zero(),
            jacobian_dot_v: na::zero()
        }
    }
}


impl<N: RealField> Joint<N> for SlotJoint<N> {
    #[inline]
    fn ndofs(&self) -> usize {
        ANGULAR_DIM + 1
    }

    fn body_to_parent(&self) -> Isometry<N> {
        let trans = Translation3::from(self.shift + &*self.axis * self.pos);
        let rot   = 
        Isometry::from_parts(trans, self.rot)
    }

    fn update_jacobians(&mut self, vels: &[N]) {
        let parent_shift = self.rot * self.shift;
        let angvel       = VectorSlice3::new(vels);

        self.jacobian_v     = parent_shift.gcross_matrix_tr();
        self.jacobian_dot_v = angvel.cross(&parent_shift).gcross_matrix_tr();
    }

    fn jacobian(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>) {
        // FIXME: could we avoid the computation of rotation matrix on each `jacobian_*`  ?
        let rotmat = transform.rotation.to_rotation_matrix();
        out.fixed_rows_mut::<U3>(0).copy_from(&(rotmat * self.jacobian_v));
        out.fixed_rows_mut::<U3>(3).copy_from(rotmat.matrix());
    }

    fn jacobian_dot(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>) {
        let rotmat = transform.rotation.to_rotation_matrix();
        out.fixed_rows_mut::<U3>(0).copy_from(&(rotmat * self.jacobian_dot_v));
    }

    fn jacobian_dot_veldiff_mul_coordinates(&self, transform: &Isometry<N>, acc: &[N],
                                            out: &mut JacobianSliceMut<N>) {
        let angvel = Vector::from_row_slice(&acc[.. 3]);
        let rotmat = transform.rotation.to_rotation_matrix();
        let res    = rotmat * angvel.gcross_matrix() * self.jacobian_v;
        out.fixed_rows_mut::<U3>(0).copy_from(&res);
    }

    fn jacobian_mul_coordinates(&self, acc: &[N]) -> Velocity<N> {
        let angvel = Vector::from_row_slice(&acc[.. 3]);
        let linvel = self.jacobian_v * angvel;
        Velocity::new(linvel, angvel)
    }

    fn jacobian_dot_mul_coordinates(&self, acc: &[N]) -> Velocity<N> {
        let angvel = Vector::from_row_slice(&acc[.. 3]);
        let linvel = self.jacobian_dot_v * angvel;
        Velocity::new(linvel, na::zero())
    }

    fn integrate(&mut self, dt: N, vels: &[N]) {
        let angvel = Vector::from_row_slice(&vels[.. 3]);
        let disp   = UnitQuaternion::new(angvel *  dt);
        self.rot   = disp * self.rot;
    }
}
