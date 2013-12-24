use std::unstable::intrinsics::TypeId;
use std::num::One;
use std::ptr;
use std::rand::{SeedableRng, XorShiftRng, Rng};
use std::hashmap::HashMap;
use rsfml::graphics::render_window::RenderWindow;
use nalgebra::na::Vec3;
use nphysics::aliases::dim2;
use camera::Camera;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::lines::Lines;
use simulate;

enum SceneNode<'a> {
    BallNode(Ball<'a>),
    BoxNode(Box<'a>),
    LinesNode(Lines)
}

pub struct GraphicsManager<'a> {
    rand:      XorShiftRng,
    rb2sn:     HashMap<uint, ~[SceneNode<'a>]>,
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

    pub fn simulate(builder: |&mut GraphicsManager| -> dim2::BodyWorld2d<f32>) {
        simulate::simulate(builder)
    }

    pub fn add(&mut self, body: @mut dim2::Body2d<f32>) {

        let nodes = {
            let rb = body.to_rigid_body_or_fail();
            let mut nodes = ~[];

            self.add_geom(body, One::one(), rb.geom(), &mut nodes);

            nodes
        };

        self.rb2sn.insert(ptr::to_mut_unsafe_ptr(body) as uint, nodes);
    }

    fn add_geom(&mut self,
                body:  @mut dim2::Body2d<f32>,
                delta: dim2::Transform2d<f32>,
                geom:  dim2::Geom2dRef<f32>,
                out:   &mut ~[SceneNode<'a>]) {
        type Pl = dim2::Plane2d<f32>;
        type Bl = dim2::Ball2d<f32>;
        type Bo = dim2::Box2d<f32>;
        type Cy = dim2::Cylinder2d<f32>;
        type Co = dim2::Cone2d<f32>;
        type Cm = dim2::Compound2d<f32>;
        type Ls = dim2::LineStrip2d<f32>;

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

            for &(t, ref s) in c.shapes().iter() {
                self.add_geom(body, delta * t, *s, out)
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
                 _: @mut dim2::Body2d<f32>,
                 _: &dim2::Plane2d<f32>,
                 _:  &mut ~[SceneNode]) {
    }

    fn add_ball(&mut self,
                body:  @mut dim2::Body2d<f32>,
                delta: dim2::Transform2d<f32>,
                geom:  &dim2::Ball2d<f32>,
                out:   &mut ~[SceneNode]) {
        let color = self.color_for_object(body);
        out.push(BallNode(Ball::new(body, delta, geom.radius(), color)))
    }

    fn add_lines(&mut self,
               body:  @mut dim2::Body2d<f32>,
               delta: dim2::Transform2d<f32>,
               geom:  &dim2::LineStrip2d<f32>,
               out:   &mut ~[SceneNode]) {

        let color = self.color_for_object(body);

        let vs = geom.vertices().clone();
        let is = geom.indices().clone();

        out.push(LinesNode(Lines::new(body, delta, vs, is, color)))
    }


    fn add_box(&mut self,
               body:  @mut dim2::Body2d<f32>,
               delta: dim2::Transform2d<f32>,
               geom:  &dim2::Box2d<f32>,
               out:   &mut ~[SceneNode]) {
        let rx = geom.half_extents().x;
        let ry = geom.half_extents().y;

        let color = self.color_for_object(body);

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


    pub fn color_for_object(&mut self, body: &dim2::Body2d<f32>) -> Vec3<u8> {
        let key = ptr::to_unsafe_ptr(body) as uint;
        match self.obj2color.find(&key) {
            Some(color) => return *color,
            None => { }
        }

        let color = Vec3::new(
            self.rand.gen_range(0, 256) as u8,
            self.rand.gen_range(0, 256) as u8,
            self.rand.gen_range(0, 256) as u8);


        self.obj2color.insert(key, color);

        color
    }
}
