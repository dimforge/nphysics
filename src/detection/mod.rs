pub use detection::collision::bodies_bodies::BodiesBodies;
pub use detection::collision::bodies_bodies::BodyBodyDispatcher;
pub use detection::joint::joint_manager::JointManager;
pub use detection::activation_manager::ActivationManager;

pub mod constraint;
pub mod detector;

pub mod collision {
    pub mod bodies_bodies;
}

pub mod joint {
    pub mod joint_manager;
    pub mod anchor;
    pub mod ball_in_socket;
    pub mod fixed;
}

pub mod activation_manager;
