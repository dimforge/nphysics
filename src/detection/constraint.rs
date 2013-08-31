use ncollide::contact::Contact;
use object::Body;
use detection::joint::ball_in_socket::BallInSocket;

pub enum Constraint<N, LV, AV, M, II> {
    RBRB(@mut Body<N, LV, AV, M, II>, @mut Body<N, LV, AV, M, II>, Contact<N, LV>),
    BallInSocket(@mut BallInSocket<N, LV, AV, M, II>)
}

impl<N: Clone, LV: Clone, AV, M, II> Clone for Constraint<N, LV, AV, M, II> {
    fn clone(&self) -> Constraint<N, LV, AV, M, II> {
        match *self {
            RBRB(a, b, ref c) => RBRB(a, b, c.clone()),
            BallInSocket(bis) => BallInSocket(bis),
        }
    }
}
