use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;

pub trait BodyActivationSignalHandler<O, C> {
    fn handle_body_activated_signal(&mut self, @mut O, &mut ~[C]);
    fn handle_body_deactivated_signal(&mut self, @mut O);
}

pub trait CollisionSignalHandler<O> {
    fn handle_collision_started_signal(&mut self, @mut O, @mut O);
    fn handle_collision_ended_signal(&mut self, @mut O, @mut O);
}

// FIXME: add other signals
pub struct SignalEmiter<N, O, C> {
    body_activation_handlers: HashMap<uint, @mut BodyActivationSignalHandler<O, C>, UintTWHash>,
    collision_handler:        HashMap<uint, @mut CollisionSignalHandler<O>, UintTWHash>,
}

impl<N, O, C> SignalEmiter<N, O, C> {
    pub fn new() -> SignalEmiter<N, O, C> {
        SignalEmiter {
            body_activation_handlers: HashMap::new(UintTWHash::new()),
            collision_handler:        HashMap::new(UintTWHash::new())
        }
    }
}


/*
 * Implement handler add/remove
 */
impl<N, O, C> SignalEmiter<N, O, C> {
    pub fn add_body_activation_handler(&mut self, id: uint, handler: @mut BodyActivationSignalHandler<O, C>) {
        self.body_activation_handlers.insert(id, handler);
    }

    pub fn remove_body_activation_handler(&mut self, id: uint) {
        self.body_activation_handlers.remove(&id);
    }

    pub fn add_collision_handler(&mut self, id: uint, handler: @mut CollisionSignalHandler<O>) {
        self.collision_handler.insert(id, handler);
    }

    pub fn remove_collision_handler(&mut self, id: uint) {
        self.collision_handler.remove(&id);
    }

}

/*
 * Implement signal emition
 */
impl<N, O, C> SignalEmiter<N, O, C> {
    pub fn emit_body_deactivated(&self, o: @mut O) {
        for h in self.body_activation_handlers.elements().iter() {
            h.value.handle_body_deactivated_signal(o)
        }
    }

    pub fn emit_body_activated(&self, o: @mut O, out: &mut ~[C]) {
        for h in self.body_activation_handlers.elements().iter() {
            h.value.handle_body_activated_signal(o, out)
        }
    }

    pub fn emit_collision_started(&self, o1: @mut O, o2: @mut O) {
        for h in self.collision_handler.elements().iter() {
            h.value.handle_collision_started_signal(o1, o2)
        }
    }

    pub fn emit_collision_ended(&self, o1: @mut O, o2: @mut O) {
        for h in self.collision_handler.elements().iter() {
            h.value.handle_collision_ended_signal(o1, o2)
        }
    }
}
