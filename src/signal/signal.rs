use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::{UintTWHash};

// FIXME: add other signals
pub struct SignalEmiter<N, O, C> {
    body_activated_handlers:   HashMap<uint, @fn(@mut O, &mut ~[C]), UintTWHash>,
    body_deactivated_handlers: HashMap<uint, @fn(@mut O), UintTWHash>,
    collision_started_handler:   HashMap<uint, @fn(@mut O, @mut O), UintTWHash>,
    collision_ended_handler:      HashMap<uint, @fn(@mut O, @mut O), UintTWHash>
}

impl<N, O, C> SignalEmiter<N, O, C> {
    pub fn new() -> SignalEmiter<N, O, C> {
        SignalEmiter {
            body_activated_handlers:   HashMap::new(UintTWHash::new()),
            body_deactivated_handlers: HashMap::new(UintTWHash::new()),
            collision_started_handler:   HashMap::new(UintTWHash::new()),
            collision_ended_handler:      HashMap::new(UintTWHash::new())
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

    pub fn add_collision_started_handler(&mut self, id: uint, handler: @fn(@mut O, @mut O)) {
        self.collision_started_handler.insert(id, handler);
    }

    pub fn remove_collision_started_handler(&mut self, id: uint) {
        self.collision_started_handler.remove(&id);
    }

    pub fn add_collision_ended_handler(&mut self, id: uint, handler: @fn(@mut O, @mut O)) {
        self.collision_ended_handler.insert(id, handler);
    }

    pub fn remove_collision_ended_handler(&mut self, id: uint) {
        self.collision_ended_handler.remove(&id);
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

    pub fn emit_collision_started(&self, o1: @mut O, o2: @mut O) {
        for h in self.collision_started_handler.elements().iter() {
            (h.value)(o1, o2)
        }
    }

    pub fn emit_collision_ended(&self, o1: @mut O, o2: @mut O) {
        for h in self.collision_ended_handler.elements().iter() {
            (h.value)(o1, o2)
        }
    }
}
