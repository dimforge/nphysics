use std::any::AnyRefExt;
use std::rc::Rc;
use std::cell::RefCell;
use std::intrinsics::TypeId;
use std::num::One;
use std::collections::HashMap;
use rand::{SeedableRng, XorShiftRng, Rng};
use rsfml::graphics::RenderWindow;
use nalgebra::na::{Vec3, Iso2};
use nphysics::world::World;
use nphysics::object::RigidBody;
use ncollide::geom::Geom;
use ncollide::geom;
use camera::Camera;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::lines::Lines;
use simulate;

pub enum SceneNode<'a> {
    BallNode(Ball<'a>),
    BoxNode(Box<'a>),
    LinesNode(Lines)
}

pub struct GraphicsManager<'a> {
    rand:      XorShiftRng,
    rb2sn:     HashMap<uint, Vec<SceneNode<'a>>>,
    obj2color: HashMap<uint, Vec3<u8>>
}

impl<'a> GraphicsManager<'a> {
    pub fn new() -> GraphicsManager {
        GraphicsManager {
            rand:      SeedableRng::from_seed([0, 1, 2, 3]),
            rb2sn:     HashMap::new(),
            obj2color: HashMap::new()
        }
    }

    pub fn simulate(builder: |&mut GraphicsManager| -> World) {
        simulate::simulate(builder)
    }

    pub fn add(&mut self, body: Rc<RefCell<RigidBody>>) {

        let nodes = {
            let rb    = body.borrow();
            let mut nodes = Vec::new();

            self.add_geom(body.clone(), One::one(), rb.geom_ref(), &mut nodes);

            nodes
        };

        self.rb2sn.insert(body.deref() as *RefCell<RigidBody> as uint, nodes);
    }

    fn add_geom(&mut self,
                body:  Rc<RefCell<RigidBody>>,
                delta: Iso2<f32>,
                geom:  &Geom,
                out:   &mut Vec<SceneNode<'a>>) {
        type Pl = geom::Plane;
        type Bl = geom::Ball;
        type Bo = geom::Cuboid;
        type Cy = geom::Cylinder;
        type Co = geom::Cone;
        type Cm = geom::Compound;
        type Ls = geom::Mesh;

        let id = geom.get_type_id();
        if id == TypeId::of::<Pl>(){
            self.add_plane(body, geom.as_ref::<Pl>().unwrap(), out)
        }
        else if id == TypeId::of::<Bl>() {
            self.add_ball(body, delta, geom.as_ref::<Bl>().unwrap(), out)
        }
        else if id == TypeId::of::<Bo>() {
            self.add_box(body, delta, geom.as_ref::<Bo>().unwrap(), out)
        }
        else if id == TypeId::of::<Cm>() {
            let c = geom.as_ref::<Cm>().unwrap();

            for &(t, ref s) in c.geoms().iter() {
                self.add_geom(body.clone(), delta * t, **s, out)
            }
        }
        else if id == TypeId::of::<Ls>() {
            self.add_lines(body, delta, geom.as_ref::<Ls>().unwrap(), out)
        }
        else {
            fail!("Not yet implemented.")
        }

    }

    fn add_plane(&mut self,
                 _: Rc<RefCell<RigidBody>>,
                 _: &geom::Plane,
                 _: &mut Vec<SceneNode>) {
    }

    fn add_ball(&mut self,
                body:  Rc<RefCell<RigidBody>>,
                delta: Iso2<f32>,
                geom:  &geom::Ball,
                out:   &mut Vec<SceneNode>) {
        let color = self.color_for_object(&body);
        out.push(BallNode(Ball::new(body, delta, geom.radius(), color)))
    }

    fn add_lines(&mut self,
               body:  Rc<RefCell<RigidBody>>,
               delta: Iso2<f32>,
               geom:  &geom::Mesh,
               out:   &mut Vec<SceneNode>) {

        let color = self.color_for_object(&body);

        let vs = geom.vertices().clone();
        let is = geom.indices().clone();

        out.push(LinesNode(Lines::new(body, delta, vs, is, color)))
    }


    fn add_box(&mut self,
               body:  Rc<RefCell<RigidBody>>,
               delta: Iso2<f32>,
               geom:  &geom::Cuboid,
               out:   &mut Vec<SceneNode>) {
        let rx = geom.half_extents().x;
        let ry = geom.half_extents().y;

        let color = self.color_for_object(&body);

        out.push(BoxNode(Box::new(body, delta, rx, ry, color)))
    }

    pub fn draw(&mut self, rw: &mut RenderWindow, c: &Camera) {
        c.activate_scene(rw);

        for (_, ns) in self.rb2sn.mut_iter() {
            for n in ns.mut_iter() {
                match *n {
                    BoxNode(ref mut b)   => b.update(),
                    BallNode(ref mut b)  => b.update(),
                    LinesNode(ref mut l) => l.update(),
                }
            }
        }

        for (_, ns) in self.rb2sn.mut_iter() {
            for n in ns.mut_iter() {
                match *n {
                    BoxNode(ref b)   => b.draw(rw),
                    BallNode(ref b)  => b.draw(rw),
                    LinesNode(ref l) => l.draw(rw),
                }
            }
        }

        c.activate_ui(rw);
    }


    pub fn color_for_object(&mut self, body: &Rc<RefCell<RigidBody>>) -> Vec3<u8> {
        let key = body.deref() as *RefCell<RigidBody> as uint;
        match self.obj2color.find(&key) {
            Some(color) => return *color,
            None => { }
        }

        let color = Vec3::new(
            self.rand.gen_range(0u, 256) as u8,
            self.rand.gen_range(0u, 256) as u8,
            self.rand.gen_range(0u, 256) as u8);


        self.obj2color.insert(key, color);

        color
    }
}
