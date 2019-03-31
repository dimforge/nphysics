use either::Either;

use na::{RealField, Cholesky, Dynamic, DVectorSliceMut, VectorSliceMutN, Point2, Point3, DVector, DVectorSlice};
#[cfg(feature = "dim3")]
use na::Point4;
use ncollide::shape::{Segment, Triangle};
use ncollide::query::PointQueryWithLocation;
#[cfg(feature = "dim3")]
use ncollide::shape::Tetrahedron;

use crate::object::BodyStatus;
use crate::solver::ForceDirection;
use crate::math::{Point, Isometry, Dim, DIM};


pub(crate) fn elasticity_coefficients<N: RealField>(young_modulus: N, poisson_ratio: N) -> (N, N, N) {
    let _1 = N::one();
    let _2: N = na::convert(2.0);

    let d0 = (young_modulus * (_1 - poisson_ratio)) / ((_1 + poisson_ratio) * (_1 - _2 * poisson_ratio));
    let d1 = (young_modulus * poisson_ratio) / ((_1 + poisson_ratio) * (_1 - _2 * poisson_ratio));
    let d2 = (young_modulus * (_1 - _2 * poisson_ratio)) / (_2 * (_1 + poisson_ratio) * (_1 - _2 * poisson_ratio));
    (d0, d1, d2)
}


/// Indices of the nodes of on element of a body decomposed in finite elements.
#[derive(Copy, Clone, Debug)]
pub(crate) enum FiniteElementIndices {
    #[cfg(feature = "dim3")]
    /// A tetrahedral element.
    Tetrahedron(Point4<usize>),
    /// A triangular element.
    Triangle(Point3<usize>),
    /// A segment element.
    Segment(Point2<usize>)
}

impl FiniteElementIndices {
    #[inline]
    pub fn as_slice(&self) -> &[usize] {
        match self {
            #[cfg(feature = "dim3")]
            FiniteElementIndices::Tetrahedron(idx) => idx.coords.as_slice(),
            FiniteElementIndices::Triangle(idx) => idx.coords.as_slice(),
            FiniteElementIndices::Segment(idx) => idx.coords.as_slice(),
        }
    }

//    #[inline]
//    pub fn len(&self) -> usize {
//        match self {
//            #[cfg(feature = "dim3")]
//            FiniteElementIndices::Tetrahedron(_) => 4,
//            FiniteElementIndices::Triangle(_) => 3,
//            FiniteElementIndices::Segment(_) => 2,
//        }
//    }
}


#[inline]
pub(crate) fn world_point_at_material_point<N: RealField>(indices: FiniteElementIndices, positions: &DVector<N>, point: &Point<N>) -> Point<N> {
    match indices {
        FiniteElementIndices::Segment(indices) => {
            let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
            let b = positions.fixed_rows::<Dim>(indices.y).into_owned();
            Point::from(a * (N::one() - point.x) + b * point.x)
        }
        FiniteElementIndices::Triangle(indices) => {
            let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
            let b = positions.fixed_rows::<Dim>(indices.y).into_owned();
            let c = positions.fixed_rows::<Dim>(indices.z).into_owned();
            Point::from(a * (N::one() - point.x - point.y) + b * point.x + c * point.y)
        }
        #[cfg(feature = "dim3")]
        FiniteElementIndices::Tetrahedron(indices) => {
            let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
            let b = positions.fixed_rows::<Dim>(indices.y).into_owned();
            let c = positions.fixed_rows::<Dim>(indices.z).into_owned();
            let d = positions.fixed_rows::<Dim>(indices.w).into_owned();
            Point::from(a * (N::one() - point.x - point.y - point.z) + b * point.x + c * point.y + d * point.z)
        }
    }
}

// NOTE: the barycentric coordinate with the form (1 - x - y - ...) is the first component
// because it makes it simpler to handle the case where we don't know at compile-time the
// dimension of `indices`.
#[inline]
pub(crate) fn material_point_at_world_point<N: RealField>(indices: FiniteElementIndices, positions: &DVector<N>, point: &Point<N>) -> Point<N> {
    match indices {
        FiniteElementIndices::Segment(indices) => {
            let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
            let b = positions.fixed_rows::<Dim>(indices.y).into_owned();

            let seg = Segment::new(
                Point::from(a),
                Point::from(b),
            );

            // FIXME: do we really want to project here? Even in 2D?
            let proj = seg.project_point_with_location(&Isometry::identity(), point, false).1;
            let bcoords = proj.barycentric_coordinates();

            let mut res = Point::origin();
            res.x = bcoords[1];
            res
        }
        FiniteElementIndices::Triangle(indices) => {
            let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
            let b = positions.fixed_rows::<Dim>(indices.y).into_owned();
            let c = positions.fixed_rows::<Dim>(indices.z).into_owned();

            let tri = Triangle::new(
                Point::from(a),
                Point::from(b),
                Point::from(c),
            );

            // FIXME: do we really want to project here? Even in 2D?
            let proj = tri.project_point_with_location(&Isometry::identity(), point, false).1;
            let bcoords = proj.barycentric_coordinates().unwrap();

            let mut res = Point::origin();
            res.x = bcoords[1];
            res.y = bcoords[2];
            res
        }
        #[cfg(feature = "dim3")]
        FiniteElementIndices::Tetrahedron(indices) => {
            let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
            let b = positions.fixed_rows::<Dim>(indices.y).into_owned();
            let c = positions.fixed_rows::<Dim>(indices.z).into_owned();
            let d = positions.fixed_rows::<Dim>(indices.w).into_owned();

            let tetra = Tetrahedron::new(
                Point3::from(a),
                Point3::from(b),
                Point3::from(c),
                Point3::from(d),
            );

            // FIXME: what to do if this returns `None`?
            let bcoords = tetra.barycentric_coordinates(point).unwrap_or([N::zero(); 4]);
            Point3::new(bcoords[1], bcoords[2], bcoords[3])
        }
    }
}

#[inline]
pub(crate) fn fill_contact_geometry_fem<N: RealField>(
    ndofs: usize,
    status: BodyStatus,
    indices: FiniteElementIndices,
    positions: &DVector<N>,
    velocities: &DVector<N>,
    kinematic_nodes: &DVector<bool>,
    inv_augmented_mass: Either<N, &Cholesky<N, Dynamic>>,
    // Original parameters of fill_contact_geometry.
    center: &Point<N>,
    force_dir: &ForceDirection<N>,
    j_id: usize,
    wj_id: usize,
    jacobians: &mut [N],
    inv_r: &mut N,
    ext_vels: Option<&DVectorSlice<N>>,
    out_vel: Option<&mut N>
) {
    if status == BodyStatus::Static || status == BodyStatus::Disabled {
        return;
    }

    // Needed by the non-linear SOR-prox.
    // FIXME: should this `fill` be done by the non-linear SOR-prox itself?
    if status == BodyStatus::Dynamic {
        DVectorSliceMut::from_slice(&mut jacobians[j_id..], ndofs).fill(N::zero());
    }

    if let ForceDirection::Linear(dir) = force_dir {
        match indices {
            FiniteElementIndices::Segment(indices) => {
                let kinematic1 = kinematic_nodes[indices.x / DIM];
                let kinematic2 = kinematic_nodes[indices.y / DIM];

                let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
                let b = positions.fixed_rows::<Dim>(indices.y).into_owned();

                let seg = Segment::new(
                    Point::from(a),
                    Point::from(b),
                );

                // FIXME: This is costly!
                let proj = seg.project_point_with_location(&Isometry::identity(), center, false).1;
                let bcoords = proj.barycentric_coordinates();

                let dir1 = **dir * bcoords[0];
                let dir2 = **dir * bcoords[1];

                if status == BodyStatus::Dynamic {
                    if !kinematic1 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.x..]).copy_from(&dir1);
                    }
                    if !kinematic2 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.y..]).copy_from(&dir2);
                    }
                }

                if let Some(out_vel) = out_vel {
                    let va = velocities.fixed_rows::<Dim>(indices.x);
                    let vb = velocities.fixed_rows::<Dim>(indices.y);

                    *out_vel += va.dot(&dir1) + vb.dot(&dir2);

                    if status == BodyStatus::Dynamic {
                        if let Some(ext_vels) = ext_vels {
                            if !kinematic1 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.x).dot(&dir1);
                            }
                            if !kinematic2 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.y).dot(&dir2);
                            }
                        }
                    }
                }
            }
            FiniteElementIndices::Triangle(indices) => {
                let kinematic1 = kinematic_nodes[indices.x / DIM];
                let kinematic2 = kinematic_nodes[indices.y / DIM];
                let kinematic3 = kinematic_nodes[indices.z / DIM];

                let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
                let b = positions.fixed_rows::<Dim>(indices.y).into_owned();
                let c = positions.fixed_rows::<Dim>(indices.z).into_owned();

                let tri = Triangle::new(
                    Point::from(a),
                    Point::from(b),
                    Point::from(c),
                );

                // FIXME: This is costly!
                let proj = tri.project_point_with_location(&Isometry::identity(), center, false).1;
                let bcoords = proj.barycentric_coordinates().unwrap();

                let dir1 = **dir * bcoords[0];
                let dir2 = **dir * bcoords[1];
                let dir3 = **dir * bcoords[2];

                if status == BodyStatus::Dynamic {
                    if !kinematic1 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.x..]).copy_from(&dir1);
                    }
                    if !kinematic2 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.y..]).copy_from(&dir2);
                    }
                    if !kinematic3 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.z..]).copy_from(&dir3);
                    }
                }

                if let Some(out_vel) = out_vel {
                    let va = velocities.fixed_rows::<Dim>(indices.x);
                    let vb = velocities.fixed_rows::<Dim>(indices.y);
                    let vc = velocities.fixed_rows::<Dim>(indices.z);

                    *out_vel += va.dot(&dir1) + vb.dot(&dir2) + vc.dot(&dir3);

                    if status == BodyStatus::Dynamic {
                        if let Some(ext_vels) = ext_vels {
                            if !kinematic1 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.x).dot(&dir1);
                            }
                            if !kinematic2 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.y).dot(&dir2);
                            }
                            if !kinematic3 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.z).dot(&dir3);
                            }
                        }
                    }
                }
            }
            #[cfg(feature = "dim3")]
            FiniteElementIndices::Tetrahedron(indices) => {
                let kinematic1 = kinematic_nodes[indices.x / DIM];
                let kinematic2 = kinematic_nodes[indices.y / DIM];
                let kinematic3 = kinematic_nodes[indices.z / DIM];
                let kinematic4 = kinematic_nodes[indices.w / DIM];

                let a = positions.fixed_rows::<Dim>(indices.x).into_owned();
                let b = positions.fixed_rows::<Dim>(indices.y).into_owned();
                let c = positions.fixed_rows::<Dim>(indices.z).into_owned();
                let d = positions.fixed_rows::<Dim>(indices.w).into_owned();

                let tetra = Tetrahedron::new(
                    Point3::from(a),
                    Point3::from(b),
                    Point3::from(c),
                    Point3::from(d),
                );

                // FIXME: what to do if this returns `None`?
                let bcoords = tetra.barycentric_coordinates(center).unwrap_or([N::zero(); 4]);

                let dir1 = **dir * bcoords[0];
                let dir2 = **dir * bcoords[1];
                let dir3 = **dir * bcoords[2];
                let dir4 = **dir * bcoords[3];

                if status == BodyStatus::Dynamic {
                    if !kinematic1 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.x..]).copy_from(&dir1);
                    }
                    if !kinematic2 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.y..]).copy_from(&dir2);
                    }
                    if !kinematic3 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.z..]).copy_from(&dir3);
                    }
                    if !kinematic4 {
                        VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + indices.w..]).copy_from(&dir4);
                    }
                }

                if let Some(out_vel) = out_vel {
                    let va = velocities.fixed_rows::<Dim>(indices.x);
                    let vb = velocities.fixed_rows::<Dim>(indices.y);
                    let vc = velocities.fixed_rows::<Dim>(indices.z);
                    let vd = velocities.fixed_rows::<Dim>(indices.w);

                    *out_vel += va.dot(&dir1) + vb.dot(&dir2) + vc.dot(&dir3) + vd.dot(&dir4);

                    if status == BodyStatus::Dynamic {
                        if let Some(ext_vels) = ext_vels {
                            if !kinematic1 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.x).dot(&dir1);
                            }
                            if !kinematic2 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.y).dot(&dir2);
                            }
                            if !kinematic3 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.z).dot(&dir3);
                            }
                            if !kinematic4 {
                                *out_vel += ext_vels.fixed_rows::<Dim>(indices.w).dot(&dir4);
                            }
                        }
                    }
                }
            }
        }

        if status == BodyStatus::Dynamic {
            match inv_augmented_mass {
                Either::Right(inv_augmented_mass) => {
                    // FIXME: use a mem::copy_nonoverlapping?
                    for i in 0..ndofs {
                        jacobians[wj_id + i] = jacobians[j_id + i];
                    }

                    inv_augmented_mass.solve_mut(&mut DVectorSliceMut::from_slice(&mut jacobians[wj_id..], ndofs));
                },
                Either::Left(inv_augmented_mass) => {
                    for i in 0..ndofs {
                        jacobians[wj_id + i] = jacobians[j_id + i] * inv_augmented_mass;
                    }
                }
            }

            // FIXME: optimize this because j is sparse.
            *inv_r += DVectorSlice::from_slice(&jacobians[j_id..], ndofs).dot(&DVectorSlice::from_slice(&jacobians[wj_id..], ndofs));
        }
    }
}