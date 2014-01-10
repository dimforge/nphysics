use std::rc::Rc;
use std::cell::RefCell;
use ncollide::contact::Contact;
use object::RigidBody;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;

pub enum Constraint {
    RBRB(Rc<RefCell<RigidBody>>, Rc<RefCell<RigidBody>>, Contact),
    BallInSocket(Rc<RefCell<BallInSocket>>),
    Fixed(Rc<RefCell<Fixed>>),
}

impl Clone for Constraint {
    fn clone(&self) -> Constraint {
        match *self {
            RBRB(ref a, ref b, ref c) => RBRB(a.clone(), b.clone(), c.clone()),
            BallInSocket(ref bis)     => BallInSocket(bis.clone()),
            Fixed(ref f)              => Fixed(f.clone()),
        }
    }
}
