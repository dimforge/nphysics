pub use detection::collision::bodies_bodies::BodiesBodies;
pub use BodiesBodiesDispatcher = detection::collision::bodies_bodies::Dispatcher;
pub use detection::joint::joint_manager::JointManager;
pub use detection::private::island_activation_manager::IslandActivationManager;

// modules
pub mod constraint;
pub mod detector;

pub mod collision {
    mod bodies_bodies;
}

pub mod joint {
    mod joint_manager;
    pub mod anchor;
    pub mod ball_in_socket;
    pub mod fixed;
}

mod private {
    #[path = "../island_activation_manager.rs"]
    mod island_activation_manager;
}
