use num::Zero;

use na::RealField;
use na::{Matrix1, Point2};
use na;
use ncollide::utils;
use ncollide::shape::ConvexPolygon;
use crate::volumetric::Volumetric;
use crate::math::{AngularInertia, Point};

/// The area and center of mass of a 2D convex Polyline.
///
/// The polyline is not checked to be actually convex.
pub fn convex_polyline_area_and_center_of_mass_unchecked<N: RealField>(
    convex_polyline: &[Point<N>],
) -> (N, Point<N>) {
    let geometric_center = utils::center(convex_polyline);
    let mut res = Point::origin();
    let mut areasum = N::zero();

    let mut iterpeek = convex_polyline.iter().peekable();
    let firstelement = *iterpeek.peek().unwrap(); // Stores first element to close the cycle in the end with unwrap_or.
    while let Some(elem) = iterpeek.next() {
        let area = utils::triangle_area(
            elem,
            iterpeek.peek().unwrap_or(&firstelement),
            &geometric_center,
        );
        let center = utils::triangle_center(
            elem,
            iterpeek.peek().unwrap_or(&firstelement),
            &geometric_center,
        );

        res += center.coords * area;
        areasum += area;
    }

    if areasum.is_zero() {
        (areasum, geometric_center)
    } else {
        (areasum, res / areasum)
    }
}

/// The mass properties of a 2D convex Polyline.
///
/// The polyline is not checked to be actually convex.
pub fn convex_polyline_mass_properties_unchecked<N: RealField>(
    convex_polyline: &[Point<N>],
    density: N,
) -> (N, Point<N>, N) {
    let (area, com) = convex_polyline_area_and_center_of_mass_unchecked(convex_polyline);

    if area.is_zero() {
        return (na::zero(), com, na::zero());
    }

    let mut itot = N::zero();
    let factor: N = na::convert(0.5 * 1.0 / 3.0);

    let mut iterpeek = convex_polyline.iter().peekable();
    let firstelement = *iterpeek.peek().unwrap(); // store first element to close the cycle in the end with unwrap_or
    while let Some(elem) = iterpeek.next() {
        let area = utils::triangle_area(&com, elem, iterpeek.peek().unwrap_or(&firstelement));

        // algorithm adapted from Box2D
        let e1 = *elem - com;
        let e2 = **(iterpeek.peek().unwrap_or(&firstelement)) - com;

        let ex1 = e1[0];
        let ey1 = e1[1];
        let ex2 = e2[0];
        let ey2 = e2[1];

        let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
        let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

        let ipart = factor * (intx2 + inty2);

        itot += ipart * area;
    }

    (area * density, com, itot * density)
}

/// The area of a convex polyline.
///
/// The polyline is not checked to be actually convex.
pub fn convex_polyline_area_unchecked<N: RealField>(convex_polyline: &[Point<N>]) -> N {
    let geometric_center = utils::center(convex_polyline);
    let mut areasum = N::zero();

    let mut iterpeek = convex_polyline.iter().peekable();
    let firstelement = *iterpeek.peek().unwrap(); // Store first element to close the cycle in the end with unwrap_or.
    while let Some(elem) = iterpeek.next() {
        let area = utils::triangle_area(
            elem,
            iterpeek.peek().unwrap_or(&firstelement),
            &geometric_center,
        );

        areasum += area;
    }

    areasum
}

/// The area of a convex hull.
pub fn convex_hull_area<N: RealField>(points: &[Point<N>]) -> N {
    convex_polyline_area_unchecked(points)
}

/// The volume of the convex hull of a set of points.
pub fn convex_hull_volume<N: RealField>(points: &[Point<N>]) -> N {
    convex_hull_area(points)
}

/// The center of mass of the convex hull of a set of points.
pub fn convex_hull_center_of_mass<N: RealField>(points: &[Point<N>]) -> Point<N> {
    convex_polyline_area_and_center_of_mass_unchecked(points).1
}

/// The angular inertia of the convex hull of a set of points.
pub fn convex_hull_unit_angular_inertia<N: RealField>(points: &[Point<N>]) -> AngularInertia<N> {
    let (area, _, i): (_, _, N) = convex_polyline_mass_properties_unchecked(points, na::one());

    let mut tensor = AngularInertia::zero();
    tensor[(0, 0)] = i * (N::one() / area);

    tensor
}

impl<N: RealField> Volumetric<N> for ConvexPolygon<N> {
    fn area(&self) -> N {
        convex_hull_area(self.points())
    }

    fn volume(&self) -> N {
        convex_hull_volume(self.points())
    }

    fn center_of_mass(&self) -> Point2<N> {
        convex_hull_center_of_mass(self.points())
    }

    fn unit_angular_inertia(&self) -> Matrix1<N> {
        convex_hull_unit_angular_inertia(self.points())
    }

    fn mass_properties(&self, density: N) -> (N, Point2<N>, Matrix1<N>) {
        let (r1, r2, r3) = convex_polyline_mass_properties_unchecked(self.points(), density);
        (r1, r2, Matrix1::<N>::new(r3))
    }
}

#[cfg(test)]
mod test {
    use na::{self, Matrix1, Point2, Vector2, Vector3};
    use ncollide::shape::{Cuboid, ConvexPolygon};
    use ncollide::procedural;
    use crate::volumetric::Volumetric;

    #[test]
    fn test_inertia_tensor2() {
        // square
        let a = 3.8f32; // some value for side length
        let half_a = a / 2.0;

        // real moment of inertia but divided by the area of the square
        let real_moi = a.powf(2.0) / 6.0;
        let expected = Matrix1::new(real_moi);

        // standard cuboid
        let cube = Cuboid::new(Vector2::new(half_a, half_a));

        let actual = cube.unit_angular_inertia();
        assert!(
            relative_eq!(actual, expected),
            format!(
                "Inertia values do not match: actual {:?}, expected: {:?}.",
                actual, expected
            )
        );

        // convex shape
        let geom = {
            let points = vec![
                Point2::new(half_a, half_a),
                Point2::new(-half_a, half_a),
                Point2::new(-half_a, -half_a),
                Point2::new(half_a, -half_a),
            ];
            ConvexPolygon::try_new(points).unwrap()
        };
        let actual = geom.unit_angular_inertia();
        assert!(
            relative_eq!(actual, expected),
            format!(
                "Inertia values do not match: actual {:?}, expected: {:?}.",
                actual, expected
            )
        );

        // rectangle
        let a = 2.3f32; // some values for side lengths
        let b = 6.7f32;
        let half_a = a / 2.0;
        let half_b = b / 2.0;

        // real moment of inertia but divided by the area of the rectangle
        let real_moi = (1.0 / 12.0) * (a.powf(2.0) + b.powf(2.0));
        let expected = Matrix1::new(real_moi);

        // standard cuboid
        let cube = Cuboid::new(Vector2::new(half_a, half_b));

        let actual = cube.unit_angular_inertia();
        assert!(
            relative_eq!(actual, expected),
            format!(
                "Inertia values do not match: actual {:?}, expected: {:?}.",
                actual, expected
            )
        );

        // convex shape
        let geom = {
            let points = vec![
                Point2::new(half_a, half_b),
                Point2::new(-half_a, half_b),
                Point2::new(-half_a, -half_b),
                Point2::new(half_a, -half_b),
            ];
            ConvexPolygon::try_new(points).unwrap()
        };
        let actual = geom.unit_angular_inertia();
        assert!(
            relative_eq!(actual, expected),
            format!(
                "Inertia values do not match: actual {:?}, expected: {:?}.",
                actual, expected
            )
        );

        // triangle
        let b = 6.7f32; // base length
        let h = 3.8f32; // height
        let a = 2.3f32; // offset of triangle top to base begin on the x-axis
                        // if the base line is on the x-axis and starts at 0, the the top of the triangle
                        // is at x = a, y = h

        let c_x = a + b / 3.0;
        let c_y = h / 3.0;

        // real moment of inertia but divided by the area of the triangle
        // formula taken from http://www.efunda.com/math/areas/triangle.cfm
        let area = b * h / 2.0;
        let real_moi =
            (b.powf(3.0) * h - b.powf(2.0) * h * a + b * h * a.powf(2.0) + b * h.powf(3.0))
                / (36.0 * area);
        let expected = Matrix1::new(real_moi);

        // convex shape
        let geom = {
            let points = vec![
                Point2::new(0.0 - c_x, 0.0 - c_y),
                Point2::new(b - c_x, 0.0 - c_y),
                Point2::new(a - c_x, h - c_y),
            ];
            ConvexPolygon::try_new(points).unwrap()
        };
        let actual = geom.unit_angular_inertia();
        assert!(
            relative_eq!(actual, expected),
            format!(
                "Inertia values do not match: actual {:?}, expected: {:?}.",
                actual, expected
            )
        );
    }
}
