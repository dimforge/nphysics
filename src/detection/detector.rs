use detection::activation_manager::ActivationManager;

pub trait Detector<O, I, BF> {
    fn update(&mut self, &mut BF, &mut ActivationManager);
    fn interferences(&mut self, &mut ~[I], &mut BF);
}
