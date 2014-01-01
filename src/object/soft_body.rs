use nalgebra::na;
use ncollide::math::{N, V};

#[deriving(Clone, Encodable, Decodable)]
pub struct PointMass {
    invmass:    N,
    velocity:   V,
    position:   V,
}

#[deriving(Clone, Encodable, Decodable)]
pub struct ConstraintsGeometry {
    stiffness:   N,
    rest_length: N,
    impulse:     N,
    rb1:         uint,
    rb2:         uint
}

#[deriving(Clone, Encodable, Decodable)]
pub struct SoftBody {
    acc:         V,
    points:      ~[PointMass],
    constraints: ~[ConstraintsGeometry],
    active:      bool,
    index:       int
}

impl SoftBody {
    pub fn new_from_mesh(vbuf:      ~[V],
                         ids1:      ~[i32],
                         ids2:      ~[i32],
                         invmasses: ~[N],
                         stiffness: ~[N]) -> SoftBody {
        assert!(vbuf.len() == invmasses.len(),
        "Vertex buffer and mass informations must have the same size.");

        // create points mass
        let mut points = ~[];

        for (v, m) in vbuf.iter().zip(invmasses.iter()) {
            points.push(PointMass {
                invmass:    m.clone(),
                velocity:   na::zero(),
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
                rest_length: na::norm(&(vbuf[v1] - vbuf[v2])),
                impulse:     na::zero(),
                rb1:         v1 as uint,
                rb2:         v2 as uint
            });
        }

        SoftBody {
            points:      points,
            constraints: constraints,
            acc:         na::zero(),
            index:       0,
            active:      true

        }
    }

    pub fn points<'r>(&'r mut self) -> &'r mut ~[PointMass] {
        &'r mut self.points
    }
}

impl SoftBody {
    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn activate(&mut self) {
        self.active = true;
    }

    pub fn index(&self) -> int {
        self.index
    }

    pub fn set_index(&mut self, index: int) {
        self.index = index
    }
}

impl SoftBody {
    pub fn deactivate(&mut self) {
        for pt in self.points.mut_iter() {
            pt.velocity = na::zero()
        }

        self.active = false;
    }
}
