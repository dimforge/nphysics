// XXX: implement this for 2d too.

use std::ops::{Add, Mul, IndexMut};
use num::Zero;
use na::{Outer, EigenQR, Pnt3, Mat3, Mat1, Pnt2};
use na;
use ncollide::utils;
use ncollide::procedural::{Polyline, TriMesh, IndexBuffer};
use ncollide::transformation;
use ncollide::math::{Scalar, Point, Vect};
use ncollide::shape::{Convex3, Convex2};
use volumetric::Volumetric;


fn tetrahedron_unit_inertia_tensor_wrt_point<P, I>(point: &P, p1: &P, p2: &P, p3: &P, p4: &P) -> I
    where P: Point,
          I: Zero + IndexMut<(usize, usize), Output = <P::Vect as Vect>::Scalar> {
    assert!(na::dim::<P>() == 3);

    let p1 = *p1 - *point;
    let p2 = *p2 - *point;
    let p3 = *p3 - *point;
    let p4 = *p4 - *point;

    let _frac_10: <P::Vect as Vect>::Scalar = na::cast(0.1f64);
    let _frac_20: <P::Vect as Vect>::Scalar = na::cast(0.05f64);
    let _2      : <P::Vect as Vect>::Scalar = na::cast(2.0f64);

    // Just for readability.
    let x1 = p1[0]; let y1 = p1[1]; let z1 = p1[2];
    let x2 = p2[0]; let y2 = p2[1]; let z2 = p2[2];
    let x3 = p3[0]; let y3 = p3[1]; let z3 = p3[2];
    let x4 = p4[0]; let y4 = p4[1]; let z4 = p4[2];

    let diag_x = x1 * x1 + x1 * x2 + x2 * x2 + x1 * x3 + x2 * x3 + x3 * x3 + x1 * x4 + x2 * x4 + x3 * x4 + x4 * x4;
    let diag_y = y1 * y1 + y1 * y2 + y2 * y2 + y1 * y3 + y2 * y3 + y3 * y3 + y1 * y4 + y2 * y4 + y3 * y4 + y4 * y4;
    let diag_z = z1 * z1 + z1 * z2 + z2 * z2 + z1 * z3 + z2 * z3 + z3 * z3 + z1 * z4 + z2 * z4 + z3 * z4 + z4 * z4;

    let a0 = (diag_y + diag_z) * _frac_10;
    let b0 = (diag_z + diag_x) * _frac_10;
    let c0 = (diag_x + diag_y) * _frac_10;

    let a1 = (y1 * z1 * _2 + y2 * z1      + y3 * z1      + y4 * z1 +
              y1 * z2      + y2 * z2 * _2 + y3 * z2      + y4 * z2 +
              y1 * z3      + y2 * z3      + y3 * z3 * _2 + y4 * z3 +
              y1 * z4      + y2 * z4      + y3 * z4      + y4 * z4 * _2) * _frac_20;
    let b1 = (x1 * z1 * _2 + x2 * z1      + x3 * z1      + x4 * z1 +
              x1 * z2      + x2 * z2 * _2 + x3 * z2      + x4 * z2 +
              x1 * z3      + x2 * z3      + x3 * z3 * _2 + x4 * z3 +
              x1 * z4      + x2 * z4      + x3 * z4      + x4 * z4 * _2) * _frac_20;
    let c1 = (x1 * y1 * _2 + x2 * y1      + x3 * y1      + x4 * y1 +
              x1 * y2      + x2 * y2 * _2 + x3 * y2      + x4 * y2 +
              x1 * y3      + x2 * y3      + x3 * y3 * _2 + x4 * y3 +
              x1 * y4      + x2 * y4      + x3 * y4      + x4 * y4 * _2) * _frac_20;

    let mut res = na::zero::<I>();

    res[(0, 0)] =  a0; res[(0, 1)] = -b1; res[(0, 2)] = -c1;
    res[(1, 0)] = -b1; res[(1, 1)] =  b0; res[(1, 2)] = -a1;
    res[(2, 0)] = -c1; res[(2, 1)] = -a1; res[(2, 2)] =  c0;

    res
}

/// The volume and center of mass of a convex mesh.
///
/// This is unsafe as the mesh is not checked to be actually convex.
pub unsafe fn convex_mesh_volume_and_center_of_mass<P>(convex_mesh: &TriMesh<P>) -> (<P::Vect as Vect>::Scalar, P)
    where P: Point {
    let geometric_center = utils::center(&convex_mesh.coords[..]);

    let mut res = na::orig::<P>();
    let mut vol = na::zero::<<P::Vect as Vect>::Scalar>();

    match convex_mesh.indices {
        IndexBuffer::Unified(ref idx) => {
            for t in idx.iter() {
                let p2 = &convex_mesh.coords[t.x as usize];
                let p3 = &convex_mesh.coords[t.y as usize];
                let p4 = &convex_mesh.coords[t.z as usize];

                let volume = utils::tetrahedron_volume(&geometric_center, p2, p3, p4);
                let center = utils::tetrahedron_center(&geometric_center, p2, p3, p4);

                res = res + *center.as_vec() * volume;
                vol = vol + volume;
            }
        },
        IndexBuffer::Split(_) => unreachable!()
    }

    if na::is_zero(&vol) {
        (vol, geometric_center)
    }
    else {
        (vol, res / vol)
    }
}

/// The area and center of mass of a 2D convex Polyline.
///
/// This is unsafe as the mesh is not checked to be actually convex.
pub unsafe fn convex_mesh_area_and_center_of_mass2<P>(convex_mesh: &Polyline<P>) -> (<P::Vect as Vect>::Scalar, P)
    where P: Point {
    let geometric_center = utils::center(&convex_mesh.coords[..]);
    let mut res = na::orig::<P>();
    let mut areasum = na::zero::<<P::Vect as Vect>::Scalar>();

    let mut iterpeek = convex_mesh.coords.iter().peekable();
    let firstelement = *iterpeek.peek().unwrap(); // store first element to close the cycle in the end with unwrap_or
    while let Some(elem) = iterpeek.next() {
        let area = utils::triangle_area(elem, iterpeek.peek().unwrap_or(&firstelement), &geometric_center);
        let center = utils::triangle_center(elem, iterpeek.peek().unwrap_or(&firstelement), &geometric_center);

        res = res + *center.as_vec() * area;
        areasum = areasum + area;
    }

    if na::is_zero(&areasum) {
        (areasum, geometric_center)
    }
    else {
        (areasum, res / areasum)
    }
}

/// The mass properties of a convex mesh.
///
/// This is unsafe as the mesh is not checked to be actually convex.
pub unsafe fn convex_mesh_mass_properties<P, I>(convex_mesh: &TriMesh<P>,
                                                density:     <P::Vect as Vect>::Scalar)
                                                -> (<P::Vect as Vect>::Scalar, P, I)
    where P: Point,
          I: Zero +
             Add<I, Output = I> +
             Mul<<P::Vect as Vect>::Scalar, Output = I> +
             IndexMut<(usize, usize), Output = <P::Vect as Vect>::Scalar> {
    assert!(na::dim::<P>() == 3);

    let (volume, com) = convex_mesh_volume_and_center_of_mass(convex_mesh);

    if na::is_zero(&volume) {
        return (na::zero(), com, na::zero());
    }

    let mut itot = na::zero::<I>();

    match convex_mesh.indices {
        IndexBuffer::Unified(ref idx) => {
            for t in idx.iter() {
                let p2 = &convex_mesh.coords[t.x as usize];
                let p3 = &convex_mesh.coords[t.y as usize];
                let p4 = &convex_mesh.coords[t.z as usize];

                let vol      = utils::tetrahedron_volume(&com, p2, p3, p4);
                let ipart: I = tetrahedron_unit_inertia_tensor_wrt_point(&com, &com, p2, p3, p4);

                itot = itot + ipart * vol;
            }
        },
        IndexBuffer::Split(_) => unreachable!()
    }

    (volume * density, com, itot * density)
}

/// The mass properties of a 2D convex Polyline.
///
/// This is unsafe as the mesh is not checked to be actually convex.
pub unsafe fn convex_mesh_mass_properties2<P>(convex_mesh: &Polyline<P>,
                                              density:     <P::Vect as Vect>::Scalar)
                                              -> (<P::Vect as Vect>::Scalar, P, <P::Vect as Vect>::Scalar)
    where P: Point {
    assert!(na::dim::<P>() == 2);

    let (area, com) = convex_mesh_area_and_center_of_mass2(convex_mesh);

    if na::is_zero(&area) {
        return (na::zero(), com, na::zero());
    }

    let mut itot = na::zero::<<P::Vect as Vect>::Scalar>();
    let factor: <P::Vect as Vect>::Scalar = na::cast(0.5 * 1.0/3.0);

    let mut iterpeek = convex_mesh.coords.iter().peekable();
    let firstelement = *iterpeek.peek().unwrap(); // store first element to close the cycle in the end with unwrap_or
    while let Some (elem) = iterpeek.next() {
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

        itot = itot + ipart * area;
    }

    (area * density, com, itot * density)
}

/// The surface of a convex mesh.
///
/// This is unsafe as the mesh is not checked to be actually convex.
pub unsafe fn convex_mesh_surface<P>(convex_mesh: &TriMesh<P>) -> <P::Vect as Vect>::Scalar
    where P: Point {
    let mut surface = na::zero::<<P::Vect as Vect>::Scalar>();

    match convex_mesh.indices {
        IndexBuffer::Unified(ref idx) => {
            for t in idx.iter() {
                let p1 = &convex_mesh.coords[t.x as usize];
                let p2 = &convex_mesh.coords[t.y as usize];
                let p3 = &convex_mesh.coords[t.z as usize];

                surface = surface + utils::triangle_area(p1, p2, p3);
            }
        },
        IndexBuffer::Split(_) => unreachable!()
    }

    surface
}

/// The surface of a convex hull.
pub fn convex_hull_surface<P>(dim: usize, points: &[P]) -> <P::Vect as Vect>::Scalar
    where P: Point,
          P::Vect: Outer,
          <P::Vect as Outer>::OuterProductType: EigenQR<<P::Vect as Vect>::Scalar, P::Vect> +
                                                Mul<P, Output = P> +
                                                Add<<P::Vect as Outer>::OuterProductType, Output = <P::Vect as Outer>::OuterProductType> +
                                                Zero + Copy {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            unimplemented!()
        }
        3 => {
            let convex_mesh = transformation::convex_hull3(points);
            unsafe { convex_mesh_surface(&convex_mesh) }
        }
        _ => {
            unimplemented!()
        }
    }
}

/// The volume of the convex hull of a set of points.
pub fn convex_hull_volume<P>(dim: usize, points: &[P]) -> <P::Vect as Vect>::Scalar
    where P: Point,
          P::Vect: Outer,
          <P::Vect as Outer>::OuterProductType: EigenQR<<P::Vect as Vect>::Scalar, P::Vect> +
                                                Mul<P, Output = P> +
                                                Add<<P::Vect as Outer>::OuterProductType, Output = <P::Vect as Outer>::OuterProductType> +
                                                Zero + Copy {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            unimplemented!()
        }
        3 => {
            let convex_mesh = transformation::convex_hull3(points);
            unsafe { convex_mesh_volume_and_center_of_mass(&convex_mesh).0 }
        }
        _ => {
            unimplemented!()
        }
    }
}

/// The center of mass of the convex hull of a set of points.
pub fn convex_hull_center_of_mass<P>(dim: usize, points: &[P]) -> P
    where P: Point,
          P::Vect: Outer,
          <P::Vect as Outer>::OuterProductType: EigenQR<<P::Vect as Vect>::Scalar, P::Vect> +
                                                Mul<P, Output = P> +
                                                Add<<P::Vect as Outer>::OuterProductType, Output = <P::Vect as Outer>::OuterProductType> +
                                                Zero + Copy {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            unimplemented!()
        }
        3 => {
            let convex_mesh = transformation::convex_hull3(points);
            unsafe { convex_mesh_volume_and_center_of_mass(&convex_mesh).1 }
        }
        _ => {
            unimplemented!()
        }
    }
}

/// The angular inertia of the convex hull of a set of points.
pub fn convex_hull_unit_angular_inertia<P, I>(dim: usize, points: &[P]) -> I
    where P: Point,
          I: Zero +
             Add<I, Output = I> +
             Mul<<P::Vect as Vect>::Scalar, Output = I> +
             IndexMut<(usize, usize), Output = <P::Vect as Vect>::Scalar>,
          P::Vect: Outer,
          <P::Vect as Outer>::OuterProductType: EigenQR<<P::Vect as Vect>::Scalar, P::Vect> +
                                                Mul<P, Output = P> +
                                                Add<<P::Vect as Outer>::OuterProductType, Output = <P::Vect as Outer>::OuterProductType> +
                                                Zero + Copy {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            let convex_mesh = transformation::convex_hull2(points);
            let (area, _, i): (_, _, <P::Vect as Vect>::Scalar) =
                               unsafe { convex_mesh_mass_properties2(&convex_mesh, na::one()) };
            let mut tensor: I = na::zero();
            tensor[(0, 0)] = i * (na::one::<<P::Vect as Vect>::Scalar>() / area);

            tensor
        }
        3 => {
            let convex_mesh = transformation::convex_hull3(points);
            unsafe {
                let (vol, _, i): (_, _, I) = convex_mesh_mass_properties(&convex_mesh, na::one());

                i * (na::one::<<P::Vect as Vect>::Scalar>() / vol)
            }
        }
        _ => {
            unimplemented!()
        }
    }
}

impl<N: Scalar> Volumetric<N, Pnt3<N>, Mat3<N>> for Convex3<N> {
    fn surface(&self) -> N {
        convex_hull_surface(3, self.points())
    }

    fn volume(&self) -> N {
        convex_hull_volume(3, self.points())
    }

    fn center_of_mass(&self) -> Pnt3<N> {
        convex_hull_center_of_mass(3, self.points())
    }

    fn unit_angular_inertia(&self) -> Mat3<N> {
        convex_hull_unit_angular_inertia(3, self.points())
    }

    fn mass_properties(&self, density: N) -> (N, Pnt3<N>, Mat3<N>) {
        let convex_mesh = transformation::convex_hull3(self.points());
        unsafe { convex_mesh_mass_properties(&convex_mesh, density) }
    }
}

impl<N: Scalar> Volumetric<N, Pnt2<N>, Mat1<N>> for Convex2<N> {
    fn surface(&self) -> N {
        convex_hull_surface(2, self.points())
    }

    fn volume(&self) -> N {
        convex_hull_volume(2, self.points())
    }

    fn center_of_mass(&self) -> Pnt2<N> {
        convex_hull_center_of_mass(2, self.points())
    }

    fn unit_angular_inertia(&self) -> Mat1<N> {
        convex_hull_unit_angular_inertia(2, self.points())
    }

    fn mass_properties(&self, density: N) -> (N, Pnt2<N>, Mat1<N>) {
        let convex_mesh = transformation::convex_hull2(self.points());
        let (r1, r2, r3) = unsafe { convex_mesh_mass_properties2(&convex_mesh, density) };
        (r1, r2, Mat1::<N>::new(r3))
    }
}

#[cfg(test)]
mod test {
    use na::{Vec2, Vec3, Mat1, Pnt2};
    use na;
    use ncollide::shape::{Convex, Cuboid};
    use ncollide::procedural;
    use volumetric::Volumetric;

    #[test]
    fn test_inertia_tensor() {
        let excentricity = 10.0;

        let mut shape = procedural::cuboid(&Vec3::new(2.0f64 - 0.08, 2.0 - 0.08, 2.0 - 0.08));

        for c in shape.coords.iter_mut() {
            c.x = c.x + excentricity;
            c.y = c.y + excentricity;
            c.z = c.z + excentricity;
        }

        let convex = Convex::new(shape.coords);
        let cuboid = Cuboid::new(Vec3::new(0.96f64, 0.96, 0.96));

        let actual   = convex.unit_angular_inertia();
        let expected = cuboid.unit_angular_inertia();

        assert!(na::approx_eq(&actual, &expected),
                format!("Inertia tensors do not match: actual {:?}, expected: {:?}.", actual, expected));

        let (actual_m, _, actual_i) = convex.mass_properties(2.37689);
        let (expected_m, _, expected_i) = cuboid.mass_properties(2.37689);

        assert!(na::approx_eq(&actual, &expected),
                format!("Unit inertia tensors do not match: actual {:?}, expected: {:?}.", actual, expected));

        assert!(na::approx_eq(&actual_i, &expected_i),
                format!("Inertia tensors do not match: actual {:?}, expected: {:?}.", actual_i, expected_i));

        assert!(na::approx_eq(&actual_m, &expected_m),
                format!("Masses do not match: actual {}, expected: {}.", actual_m, expected_m));
    }

    #[test]
    fn test_inertia_tensor2() {
        // square
        let a = 3.8f32; // some value for side length
        let half_a = a / 2.0;

        // real moment of inertia but divided by the area of the square
        let real_moi = a.powf(2.0) / 6.0;
        let expected = Mat1::new(real_moi);

        // standard cuboid
        let cube = Cuboid::new(Vec2::new(half_a, half_a));
    
        let actual = cube.unit_angular_inertia();
        assert!(na::approx_eq(&actual, &expected),
                format!("Inertia values do not match: actual {:?}, expected: {:?}.", actual, expected));

        // convex shape
        let geom = {
            let points = vec![Pnt2::new(half_a,  half_a), Pnt2::new(-half_a,  half_a),
                              Pnt2::new(-half_a, -half_a), Pnt2::new(half_a, -half_a) ];
            Convex::new(points)
        };
        let actual = geom.unit_angular_inertia();
        assert!(na::approx_eq(&actual, &expected),
                format!("Inertia values do not match: actual {:?}, expected: {:?}.", actual, expected));

        // rectangle
        let a = 2.3f32; // some values for side lengths
        let b = 6.7f32;
        let half_a = a / 2.0;
        let half_b = b / 2.0;

        // real moment of inertia but divided by the area of the rectangle
        let real_moi = (1.0 / 12.0) * (a.powf(2.0) + b.powf(2.0));
        let expected = Mat1::new(real_moi);
    
        // standard cuboid
        let cube = Cuboid::new(Vec2::new(half_a, half_b));
    
        let actual = cube.unit_angular_inertia();
        assert!(na::approx_eq(&actual, &expected),
                format!("Inertia values do not match: actual {:?}, expected: {:?}.", actual, expected));

        // convex shape
        let geom = {
            let points = vec![Pnt2::new(half_a,  half_b), Pnt2::new(-half_a,  half_b),
                              Pnt2::new(-half_a, -half_b), Pnt2::new(half_a, -half_b) ];
            Convex::new(points)
        };
        let actual = geom.unit_angular_inertia();
        assert!(na::approx_eq(&actual, &expected),
                format!("Inertia values do not match: actual {:?}, expected: {:?}.", actual, expected));


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
        let real_moi = (b.powf(3.0) * h - b.powf(2.0) * h * a + b * h * a.powf(2.0) + b * h.powf(3.0)) / (36.0 * area);
        let expected = Mat1::new(real_moi);

        // convex shape
        let geom = {
            let points = vec![Pnt2::new(0.0 - c_x, 0.0 - c_y), Pnt2::new(b - c_x, 0.0 - c_y),
                              Pnt2::new(a - c_x, h - c_y) ];
            Convex::new(points)
        };
        let actual = geom.unit_angular_inertia();
        assert!(na::approx_eq(&actual, &expected),
                format!("Inertia values do not match: actual {:?}, expected: {:?}.", actual, expected));
    }
}
