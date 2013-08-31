pub use world::private::world::World;
pub use world::private::body_world::BodyWorld;

mod private {
    #[path = "../world.rs"]
    mod world;

    #[path = "../body_world.rs"]
    mod body_world;
}
