use std::rc::Rc;
use std::cell::RefCell;
use std::collections::HashMap;
use rand::{SeedableRng, XorShiftRng, Rng};
use sfml::graphics::RenderWindow;
use na::{Pnt3, Iso2};
use na;
use nphysics::object::RigidBody;
use ncollide::inspection::Repr2;
use ncollide::shape;
use camera::Camera;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::lines::Lines;
use objects::segment::Segment;

pub enum SceneNode<'a> {
    BallNode(Ball<'a>),
    BoxNode(Box<'a>),
    LinesNode(Lines),
    SegmentNode(Segment)
}

impl<'a> SceneNode<'a> {
    pub fn select(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n) => n.select(),
            SceneNode::BoxNode(ref mut n) => n.select(),
            SceneNode::LinesNode(ref mut n) => n.select(),
            SceneNode::SegmentNode(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n) => n.unselect(),
            SceneNode::BoxNode(ref mut n) => n.unselect(),
            SceneNode::LinesNode(ref mut n) => n.unselect(),
            SceneNode::SegmentNode(ref mut n) => n.unselect(),
        }
    }
}

pub struct GraphicsManager<'a> {
    rand:      XorShiftRng,
    rb2sn:     HashMap<usize, Vec<SceneNode<'a>>>,
    obj2color: HashMap<usize, Pnt3<u8>>
}

impl<'a> GraphicsManager<'a> {
    pub fn new() -> GraphicsManager<'a> {
        GraphicsManager {
            rand:      SeedableRng::from_seed([0, 1, 2, 3]),
            rb2sn:     HashMap::new(),
            obj2color: HashMap::new()
        }
    }

    pub fn add(&mut self, body: Rc<RefCell<RigidBody>>) {

        let nodes = {
            let rb    = body.borrow();
            let mut nodes = Vec::new();

            self.add_shape(body.clone(), na::one(), rb.shape_ref(), &mut nodes);

            nodes
        };

        self.rb2sn.insert(&*body as *const RefCell<RigidBody> as usize, nodes);
    }

    fn add_shape(&mut self,
                 body:  Rc<RefCell<RigidBody>>,
                 delta: Iso2<f32>,
                 shape: &Repr2<f32>,
                 out:   &mut Vec<SceneNode<'a>>) {
        type Pl = shape::Plane2<f32>;
        type Bl = shape::Ball2<f32>;
        type Bo = shape::Cuboid2<f32>;
        type Cy = shape::Cylinder2<f32>;
        type Co = shape::Cone2<f32>;
        type Cm = shape::Compound2<f32>;
        type Ls = shape::Polyline2<f32>;
        type Se = shape::Segment2<f32>;

        let repr = shape.repr();

        if let Some(s) = repr.downcast_ref::<Pl>() {
            self.add_plane(body, s, out)
        }
        else if let Some(s) = repr.downcast_ref::<Bl>() {
            self.add_ball(body, delta, s, out)
        }
        else if let Some(s) = repr.downcast_ref::<Bo>() {
            self.add_box(body, delta, s, out)
        }
        else if let Some(s) = repr.downcast_ref::<Se>() {
            self.add_segment(body, delta, s, out)
        }
        else if let Some(s) = repr.downcast_ref::<Cm>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_shape(body.clone(), delta * t, &***s, out)
            }
        }
        else if let Some(s) = repr.downcast_ref::<Ls>() {
            self.add_lines(body, delta, s, out)
        }
        else {
            panic!("Not yet implemented.")
        }

    }

    fn add_plane(&mut self,
                 _: Rc<RefCell<RigidBody>>,
                 _: &shape::Plane2<f32>,
                 _: &mut Vec<SceneNode>) {
    }

    fn add_ball(&mut self,
                body:  Rc<RefCell<RigidBody>>,
                delta: Iso2<f32>,
                shape: &shape::Ball2<f32>,
                out:   &mut Vec<SceneNode>) {
        let color = self.color_for_object(&body);
        let margin = body.borrow().margin();
        out.push(SceneNode::BallNode(Ball::new(body, delta, shape.radius() + margin, color)))
    }

    fn add_lines(&mut self,
                 body:  Rc<RefCell<RigidBody>>,
                 delta: Iso2<f32>,
                 shape: &shape::Polyline2<f32>,
                 out:   &mut Vec<SceneNode>) {

        let color = self.color_for_object(&body);

        let vs = shape.vertices().clone();
        let is = shape.indices().clone();

        out.push(SceneNode::LinesNode(Lines::new(body, delta, vs, is, color)))
    }


    fn add_box(&mut self,
               body:  Rc<RefCell<RigidBody>>,
               delta: Iso2<f32>,
               shape: &shape::Cuboid2<f32>,
               out:   &mut Vec<SceneNode>) {
        let rx = shape.half_extents().x;
        let ry = shape.half_extents().y;
        let margin = body.borrow().margin();

        let color = self.color_for_object(&body);

        out.push(SceneNode::BoxNode(Box::new(body, delta, rx + margin, ry + margin, color)))
    }

    fn add_segment(&mut self,
                   body:  Rc<RefCell<RigidBody>>,
                   delta: Iso2<f32>,
                   shape: &shape::Segment2<f32>,
                   out:   &mut Vec<SceneNode>) {
        let a = shape.a();
        let b = shape.b();

        let color = self.color_for_object(&body);

        out.push(SceneNode::SegmentNode(Segment::new(body, delta, *a, *b, color)))
    }


    pub fn clear(&mut self) {
        self.rb2sn.clear();
    }

    pub fn draw(&mut self, rw: &mut RenderWindow, c: &Camera) {
        c.activate_scene(rw);

        for (_, ns) in self.rb2sn.iter_mut() {
            for n in ns.iter_mut() {
                match *n {
                    SceneNode::BoxNode(ref mut n) => n.update(),
                    SceneNode::BallNode(ref mut n) => n.update(),
                    SceneNode::LinesNode(ref mut n) => n.update(),
                    SceneNode::SegmentNode(ref mut n) => n.update(),
                }
            }
        }

        for (_, ns) in self.rb2sn.iter_mut() {
            for n in ns.iter_mut() {
                match *n {
                    SceneNode::BoxNode(ref n) => n.draw(rw),
                    SceneNode::BallNode(ref n) => n.draw(rw),
                    SceneNode::LinesNode(ref n) => n.draw(rw),
                    SceneNode::SegmentNode(ref n) => n.draw(rw),
                }
            }
        }

        c.activate_ui(rw);
    }

    pub fn set_color(&mut self, body: &Rc<RefCell<RigidBody>>, color: Pnt3<u8>) {
        let key = &**body as *const RefCell<RigidBody> as usize;
        self.obj2color.insert(key, color);
    }

    pub fn color_for_object(&mut self, body: &Rc<RefCell<RigidBody>>) -> Pnt3<u8> {
        let key = &**body as *const RefCell<RigidBody> as usize;
        match self.obj2color.get(&key) {
            Some(color) => return *color,
            None => { }
        }

        let color = Pnt3::new(
            self.rand.gen_range(0usize, 256) as u8,
            self.rand.gen_range(0usize, 256) as u8,
            self.rand.gen_range(0usize, 256) as u8);


        self.obj2color.insert(key, color);

        color
    }

    pub fn body_to_scene_node(&mut self, rb: &Rc<RefCell<RigidBody>>) -> Option<&mut Vec<SceneNode<'a>>> {
        self.rb2sn.get_mut(&(&**rb as *const RefCell<RigidBody> as usize))
    }
}
