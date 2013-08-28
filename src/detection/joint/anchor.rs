use object::body::Body;

pub struct Anchor<N, LV, AV, M, II> {
    body:     Option<@mut Body<N, LV, AV, M, II>>,
    position: LV
}
