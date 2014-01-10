use ncollide::math::M;
use detection::joint::anchor::Anchor;

pub struct Fixed {
    priv up_to_date: bool,
    priv anchor1:    Anchor<M>,
    priv anchor2:    Anchor<M>,
}

impl Fixed {
    pub fn new(anchor1: Anchor<M>, anchor2: Anchor<M>) -> Fixed {
        Fixed {
            up_to_date: false,
            anchor1:    anchor1,
            anchor2:    anchor2
        }
    }

    pub fn up_to_date(&self) -> bool {
        self.up_to_date
    }

    pub fn update(&mut self) {
        self.up_to_date = true
    }

    pub fn anchor1<'r>(&'r self) -> &'r Anchor<M> {
        &self.anchor1
    }

    pub fn anchor2<'r>(&'r self) -> &'r Anchor<M> {
        &self.anchor2
    }

    pub fn set_local1(&mut self, local1: M) {
        if local1 != self.anchor1.position {
            self.up_to_date = false;
            self.anchor1.position = local1
        }
    }

    pub fn set_local2(&mut self, local2: M) {
        if local2 != self.anchor2.position {
            self.up_to_date = false;
            self.anchor2.position = local2
        }
    }

    pub fn anchor1_pos(&self) -> M {
        match self.anchor1.body {
            Some(ref b) => {
                let bb = b.borrow().borrow();
                bb.get().transform_ref() * self.anchor1.position
            },
            None => self.anchor1.position.clone()
        }
    }

    pub fn anchor2_pos(&self) -> M {
        match self.anchor2.body {
            Some(ref b) => {
                let bb = b.borrow().borrow();
                bb.get().transform_ref() * self.anchor2.position
            },
            None => self.anchor2.position.clone()
        }
    }
}
