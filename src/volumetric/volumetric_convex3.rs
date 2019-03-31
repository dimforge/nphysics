use num::Zero;

use na::RealField;
use na::{Matrix3, Point3};
use na;
use ncollide::utils;
use ncollide::procedural::{IndexBuffer, TriMesh};
use ncollide::transformation;
use ncollide::shape::ConvexHull;
use crate::volumetric::Volumetric;
use crate::math::{AngularInertia, Point};

fn tetrahedron_unit_inertia_tensor_wrt_point<N: RealField>(
    point: &Point<N>,
    p1: &Point<N>,
    p2: &Point<N>,
    p3: &Point<N>,
    p4: &Point<N>,
) -> AngularInertia<N> {
    let p1 = *p1 - *point;
    let p2 = *p2 - *point;
    let p3 = *p3 - *point;
    let p4 = *p4 - *point;

    let _frac_10: N = na::convert(0.1f64);
    let _frac_20: N = na::convert(0.05f64);
    let _2: N = na::convert(2.0f64);

    // Just for readability.
    let x1 = p1[0];
    let y1 = p1[1];
    let z1 = p1[2];
    let x2 = p2[0];
    let y2 = p2[1];
    let z2 = p2[2];
    let x3 = p3[0];
    let y3 = p3[1];
    let z3 = p3[2];
    let x4 = p4[0];
    let y4 = p4[1];
    let z4 = p4[2];

    let diag_x = x1 * x1
        + x1 * x2
        + x2 * x2
        + x1 * x3
        + x2 * x3
        + x3 * x3
        + x1 * x4
        + x2 * x4
        + x3 * x4
        + x4 * x4;
    let diag_y = y1 * y1
        + y1 * y2
        + y2 * y2
        + y1 * y3
        + y2 * y3
        + y3 * y3
        + y1 * y4
        + y2 * y4
        + y3 * y4
        + y4 * y4;
    let diag_z = z1 * z1
        + z1 * z2
        + z2 * z2
        + z1 * z3
        + z2 * z3
        + z3 * z3
        + z1 * z4
        + z2 * z4
        + z3 * z4
        + z4 * z4;

    let a0 = (diag_y + diag_z) * _frac_10;
    let b0 = (diag_z + diag_x) * _frac_10;
    let c0 = (diag_x + diag_y) * _frac_10;

    let a1 = (y1 * z1 * _2
        + y2 * z1
        + y3 * z1
        + y4 * z1
        + y1 * z2
        + y2 * z2 * _2
        + y3 * z2
        + y4 * z2
        + y1 * z3
        + y2 * z3
        + y3 * z3 * _2
        + y4 * z3
        + y1 * z4
        + y2 * z4
        + y3 * z4
        + y4 * z4 * _2)
        * _frac_20;
    let b1 = (x1 * z1 * _2
        + x2 * z1
        + x3 * z1
        + x4 * z1
        + x1 * z2
        + x2 * z2 * _2
        + x3 * z2
        + x4 * z2
        + x1 * z3
        + x2 * z3
        + x3 * z3 * _2
        + x4 * z3
        + x1 * z4
        + x2 * z4
        + x3 * z4
        + x4 * z4 * _2)
        * _frac_20;
    let c1 = (x1 * y1 * _2
        + x2 * y1
        + x3 * y1
        + x4 * y1
        + x1 * y2
        + x2 * y2 * _2
        + x3 * y2
        + x4 * y2
        + x1 * y3
        + x2 * y3
        + x3 * y3 * _2
        + x4 * y3
        + x1 * y4
        + x2 * y4
        + x3 * y4
        + x4 * y4 * _2)
        * _frac_20;

    let mut res = AngularInertia::zero();

    res[(0, 0)] = a0;
    res[(0, 1)] = -b1;
    res[(0, 2)] = -c1;
    res[(1, 0)] = -b1;
    res[(1, 1)] = b0;
    res[(1, 2)] = -a1;
    res[(2, 0)] = -c1;
    res[(2, 1)] = -a1;
    res[(2, 2)] = c0;

    res
}

/// The volume and center of mass of a 3D convex mesh.
///
/// The mesh is not checked to be actually convex.
pub fn convex_mesh_volume_and_center_of_mass_unchecked<N: RealField>(
    convex_mesh: &TriMesh<N>,
) -> (N, Point<N>) {
    let geometric_center = utils::center(&convex_mesh.coords[..]);

    let mut res = Point::origin();
    let mut vol = N::zero();

    match convex_mesh.indices {
        IndexBuffer::Unified(ref idx) => {
            for t in idx.iter() {
                let p2 = &convex_mesh.coords[t.x as usize];
                let p3 = &convex_mesh.coords[t.y as usize];
                let p4 = &convex_mesh.coords[t.z as usize];

                let volume = utils::tetrahedron_volume(&geometric_center, p2, p3, p4);
                let center = utils::tetrahedron_center(&geometric_center, p2, p3, p4);

                res += center.coords * volume;
                vol += volume;
            }
        }
        IndexBuffer::Split(_) => unreachable!(),
    }

    if vol.is_zero() {
        (vol, geometric_center)
    } else {
        (vol, res / vol)
    }
}

/// The mass properties of a convex mesh.
///
/// The mesh is not checked to be actually convex.
pub fn convex_mesh_mass_properties_unchecked<N: RealField>(
    convex_mesh: &TriMesh<N>,
    density: N,
) -> (N, Point<N>, AngularInertia<N>) {
    let (volume, com) = convex_mesh_volume_and_center_of_mass_unchecked(convex_mesh);

    if volume.is_zero() {
        return (na::zero(), com, na::zero());
    }

    let mut itot = AngularInertia::zero();

    match convex_mesh.indices {
        IndexBuffer::Unified(ref idx) => {
            for t in idx.iter() {
                let p2 = &convex_mesh.coords[t.x as usize];
                let p3 = &convex_mesh.coords[t.y as usize];
                let p4 = &convex_mesh.coords[t.z as usize];

                let vol = utils::tetrahedron_volume(&com, p2, p3, p4);
                let ipart = tetrahedron_unit_inertia_tensor_wrt_point(&com, &com, p2, p3, p4);

                itot += ipart * vol;
            }
        }
        IndexBuffer::Split(_) => unreachable!(),
    }

    (volume * density, com, itot * density)
}

/// The area of a convex mesh.
///
/// The mesh is not checked to be actually convex.
pub fn convex_mesh_area_unchecked<N: RealField>(convex_mesh: &TriMesh<N>) -> N {
    let mut area = N::zero();

    match convex_mesh.indices {
        IndexBuffer::Unified(ref idx) => {
            for t in idx.iter() {
                let p1 = &convex_mesh.coords[t.x as usize];
                let p2 = &convex_mesh.coords[t.y as usize];
                let p3 = &convex_mesh.coords[t.z as usize];

                area += utils::triangle_area(p1, p2, p3);
            }
        }
        IndexBuffer::Split(_) => unreachable!(),
    }

    area
}

/// The area of a convex hull.
pub fn convex_hull_area<N: RealField>(points: &[Point<N>]) -> N {
    let convex_mesh = transformation::convex_hull(points);
    convex_mesh_area_unchecked(&convex_mesh)
}

/// The volume of the convex hull of a set of points.
pub fn convex_hull_volume<N: RealField>(points: &[Point<N>]) -> N {
    let convex_mesh = transformation::convex_hull(points);
    convex_mesh_volume_and_center_of_mass_unchecked(&convex_mesh).0
}

/// The center of mass of the convex hull of a set of points.
pub fn convex_hull_center_of_mass<N: RealField>(points: &[Point<N>]) -> Point<N> {
    let convex_mesh = transformation::convex_hull(points);
    convex_mesh_volume_and_center_of_mass_unchecked(&convex_mesh).1
}

/// The angular inertia of the convex hull of a set of points.
pub fn convex_hull_unit_angular_inertia<N: RealField>(points: &[Point<N>]) -> AngularInertia<N> {
    let convex_mesh = transformation::convex_hull(points);
    let (vol, _, i) = convex_mesh_mass_properties_unchecked(&convex_mesh, na::one());

    i * (N::one() / vol)
}

impl<N: RealField> Volumetric<N> for ConvexHull<N> {
    fn area(&self) -> N {
        convex_hull_area(self.points())
    }

    fn volume(&self) -> N {
        convex_hull_volume(self.points())
    }

    fn center_of_mass(&self) -> Point3<N> {
        convex_hull_center_of_mass(self.points())
    }

    fn unit_angular_inertia(&self) -> Matrix3<N> {
        convex_hull_unit_angular_inertia(self.points())
    }

    fn mass_properties(&self, density: N) -> (N, Point3<N>, Matrix3<N>) {
        let convex_mesh = transformation::convex_hull(self.points());
        convex_mesh_mass_properties_unchecked(&convex_mesh, density)
    }
}

#[cfg(test)]
mod test {
    use na::Vector3;
    use ncollide::shape::{ConvexHull, Cuboid};
    use ncollide::procedural;
    use crate::volumetric::Volumetric;

    #[test]
    fn test_inertia_tensor3() {
        let excentricity = 10.0;

        let mut shape = procedural::cuboid(&Vector3::new(2.0f64 - 0.08, 2.0 - 0.08, 2.0 - 0.08));

        for c in shape.coords.iter_mut() {
            c.x = c.x + excentricity;
            c.y = c.y + excentricity;
            c.z = c.z + excentricity;
        }

        let indices: Vec<usize> = shape
            .flat_indices()
            .into_iter()
            .map(|i| i as usize)
            .collect();

        let convex = ConvexHull::try_new(shape.coords, &indices).unwrap();
        let cuboid = Cuboid::new(Vector3::new(0.96f64, 0.96, 0.96));

        let actual = convex.unit_angular_inertia();
        let expected = cuboid.unit_angular_inertia();

        assert!(
            relative_eq!(actual, expected, epsilon = 1.0e-8),
            format!(
                "Inertia tensors do not match: actual {:?}, expected: {:?}.",
                actual, expected
            )
        );

        let (actual_m, _, actual_i) = convex.mass_properties(2.37689);
        let (expected_m, _, expected_i) = cuboid.mass_properties(2.37689);

        assert!(
            relative_eq!(&actual, &expected, epsilon = 1.0e-8),
            format!(
                "Unit inertia tensors do not match: actual {:?}, expected: {:?}.",
                actual, expected
            )
        );

        assert!(
            relative_eq!(actual_i, expected_i, epsilon = 1.0e-8),
            format!(
                "Inertia tensors do not match: actual {:?}, expected: {:?}.",
                actual_i, expected_i
            )
        );

        assert!(
            relative_eq!(actual_m, expected_m, epsilon = 1.0e-8),
            format!(
                "Masses do not match: actual {}, expected: {}.",
                actual_m, expected_m
            )
        );
    }
}
