pub use detection::collision::bodies_bodies::BodiesBodies;
// pub use detection::collision::bodies_bodies_parallel::BodiesBodiesParallel;
pub use detection::collision::bodies_bodies::BodyBodyDispatcher;
// pub use BodiesBodiesDispatcherParallel = detection::collision::bodies_bodies_parallel::DispatcherParallel;
pub use detection::joint::joint_manager::JointManager;
pub use detection::island_activation_manager::IslandActivationManager;

// modules
pub mod constraint;
pub mod detector;

pub mod collision {
    pub mod bodies_bodies;
    // pub mod bodies_bodies_parallel;
}

pub mod joint {
    pub mod joint_manager;
    pub mod anchor;
    pub mod ball_in_socket;
    pub mod fixed;
}

pub mod island_activation_manager;
