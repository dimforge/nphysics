use kiss3d::window;

pub trait SceneNode {
    fn update(&mut self);
    fn draw(&self, &window::Window);
}
