use ncollide::contact::Contact;
use object::body::Body;
// use detection:joint::ball_in_socket::BallInSocket;

pub enum Constraint<N, LV, AV, M, II> {
    RBRB(@mut Body<N, LV, AV, M, II>, @mut Body<N, LV, AV, M, II>, Contact<N, LV>),
    // BallInSocket(@mut BallInSocketJoint<N, LV, AV, M, II>)
}
