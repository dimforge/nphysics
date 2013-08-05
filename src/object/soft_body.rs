use std::num::Zero;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;

pub struct PointMass<N, V> {
    invmass:    N,
    velocity:   V,
    position:   V,
}

pub struct ConstraintsGeometry<N> {
    stiffness:   N,
    rest_length: N,
    impulse:     N,
    rb1:         uint,
    rb2:         uint
}

pub struct SoftBody<N, V> {
    acc:         V,
    points:      ~[PointMass<N, V>],
    constraints: ~[ConstraintsGeometry<N>]
}

impl<N: DivisionRing + NumCast + Signed + Bounded + Eq + Ord + Clone,
     V: VectorSpace<N> + Norm<N> + Dot<N> + Clone>
     SoftBody<N, V> {
    pub fn new_from_mesh(vbuf:      ~[V],
                         ids1:      ~[i32],
                         ids2:      ~[i32],
                         invmasses: ~[N],
                         stiffness: ~[N]) -> SoftBody<N, V> {
        assert!(vbuf.len() == invmasses.len(),
        "Vertex buffer and mass informations must have the same size.");

        // create points mass
        let mut points = ~[];

        for (v, m) in vbuf.iter().zip(invmasses.iter()) {
            points.push(PointMass {
                invmass:    m.clone(),
                velocity:   Zero::zero(),
                position:   v.clone(),
            });
        }

        // create constraints
        let mut constraints = ~[];

        for i in range(0u, ids2.len()) {
            let v1 = ids1[i];
            let v2 = ids2[i];
            let s  = stiffness[i].clone();

            constraints.push(ConstraintsGeometry {
                stiffness:   s.clone(),
                rest_length: (vbuf[v1] - vbuf[v2]).norm(),
                impulse:     Zero::zero(),
                rb1:         v1 as uint,
                rb2:         v2 as uint
            });
        }

        SoftBody {
            points:      points,
            constraints: constraints,
            acc:         Zero::zero()
        }
    }

    pub fn points<'r>(&'r mut self) -> &'r mut ~[PointMass<N, V>] {
        &'r mut self.points
    }
}
