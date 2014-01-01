use ncollide::contact::Contact;
use object::Body;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;

pub enum Constraint {
    RBRB(@mut Body, @mut Body, Contact),
    BallInSocket(@mut BallInSocket),
    Fixed(@mut Fixed),
}

impl Clone for Constraint {
    fn clone(&self) -> Constraint {
        match *self {
            RBRB(a, b, ref c) => RBRB(a, b, c.clone()),
            BallInSocket(bis) => BallInSocket(bis),
            Fixed(f)          => Fixed(f),
        }
    }
}
