use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::{UintTWHash};

// FIXME: add other signals
pub struct SignalEmiter<N, O, C> {
    body_activated_handlers:   HashMap<uint, @fn(@mut O, &mut ~[C]), UintTWHash>,
    body_deactivated_handlers: HashMap<uint, @fn(@mut O), UintTWHash>
}

impl<N, O, C> SignalEmiter<N, O, C> {
    pub fn new() -> SignalEmiter<N, O, C> {
        SignalEmiter {
            body_activated_handlers:   HashMap::new(UintTWHash::new()),
            body_deactivated_handlers: HashMap::new(UintTWHash::new())
        }
    }
}


/*
 * Implement handler add/remove
 */
impl<N, O, C> SignalEmiter<N, O, C> {
    pub fn add_body_activated_handler(&mut self, id: uint, handler: @fn(@mut O, &mut ~[C])) {
        self.body_activated_handlers.insert(id, handler);
    }

    pub fn remove_body_activated_handler(&mut self, id: uint) {
        self.body_activated_handlers.remove(&id);
    }

    pub fn add_body_deactivated_handler(&mut self, id: uint, handler: @fn(@mut O)) {
        self.body_deactivated_handlers.insert(id, handler);
    }

    pub fn remove_body_deactivated_handler(&mut self, id: uint) {
        self.body_deactivated_handlers.remove(&id);
    }
}

/*
 * Implement signal emition
 */
impl<N, O, C> SignalEmiter<N, O, C> {
    pub fn emit_body_deactivated(&self, o: @mut O) {
        for h in self.body_deactivated_handlers.elements().iter() {
            (h.value)(o)
        }
    }

    pub fn emit_body_activated(&self, o: @mut O, out: &mut ~[C]) {
        for h in self.body_activated_handlers.elements().iter() {
            (h.value)(o, out)
        }
    }
}
