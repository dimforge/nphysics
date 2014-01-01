use kiss3d::window::Window;
use kiss3d::object::Object;
use kiss3d::mesh::StorageLocation;
use kiss3d::mesh;
use nalgebra::na::{Vec3, Iso3, Transformation};
use nalgebra::na;
use nphysics::object::Body;
use engine::SceneNode;

pub struct Mesh {
    priv color:      Vec3<f32>,
    priv base_color: Vec3<f32>,
    priv delta:      Iso3<f32>,
    priv gfx:        Object,
    priv body:       @mut Body
}

impl Mesh {
    pub fn new(body:     @mut Body,
               delta:    Iso3<f32>,
               vertices: ~[Vec3<f32>],
               indices:  ~[Vec3<u32>],
               color:    Vec3<f32>,
               window:   &mut Window) -> Mesh {
        let vs = StorageLocation::new(vertices, false);
        let is = StorageLocation::new(indices, false);

        let mesh = mesh::Mesh::new(vs, is, None, None, false);

        let mut res = Mesh {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_mesh(mesh, 1.0),
            body:       body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.update();

        res
    }
}

impl SceneNode for Mesh {
    fn select(&mut self) {
        self.color = Vec3::x();
    }

    fn unselect(&mut self) {
        self.color = self.base_color;
    }

    fn update(&mut self) {
        let rb = self.body.to_rigid_body_or_fail();
        if rb.is_active() {
            {
                self.gfx.set_transformation(na::transformation(rb) * self.delta);
            }

            self.gfx.set_color(self.color.x, self.color.y, self.color.z);
        }
        else {
            self.gfx.set_color(self.color.x * 0.25, self.color.y * 0.25, self.color.z * 0.25);
        }
    }

    fn object<'r>(&'r self) -> &'r Object {
        &'r self.gfx
    }
}
