use object::body::Body;

pub struct Anchor<N, LV, AV, M, II> {
    body:     Option<@mut Body<N, LV, AV, M, II>>,
    position: LV
}

impl<N, LV, AV, M, II> Anchor<N, LV, AV, M, II> {
    pub fn new(body: Option<@mut Body<N, LV, AV, M, II>>, position: LV) -> Anchor<N, LV, AV, M, II> {
        Anchor {
            body:     body,
            position: position
        }
    }
}
