use std::managed;
use ncollide::math::N;
use resolution::solver::Solver;
use integration::Integrator;
use detection::detector::Detector;

pub struct World<O, C> {
    objects:     ~[@mut O],
    integrators: ~[@mut Integrator<O>],
    detectors:   ~[@mut Detector<O, C>],
    solvers:     ~[@mut Solver<C>]
}

impl<O, C> World<O, C> {
    pub fn new() -> World<O, C> {
        World {
            objects:     ~[],
            integrators: ~[],
            detectors:   ~[],
            solvers:     ~[]
        }
    }
}

impl<O, C> World<O, C> {
    pub fn step(&mut self, dt: N) {
        //
        // Integration
        //
        for i in self.integrators.mut_iter() {
            i.update(dt.clone())
        }

        //
        // Interference detection
        //
        let mut interferences = ~[];

        for d in self.detectors.mut_iter() {
            d.update();
            d.interferences(&mut interferences);
        }

        //
        // Resolution
        //
        for s in self.solvers.mut_iter() {
            s.solve(dt.clone(), interferences);
        }
    }

    pub fn add_detector<D: 'static + Detector<O, C>>(&mut self, d: @mut D) {
        sorted_insert(&mut self.detectors, d as @mut Detector<O, C>, |a, b| a.priority() < b.priority());

        for o in self.objects.iter() {
            d.add(*o)
        }
    }

    pub fn add_integrator<I: 'static + Integrator<O>>(&mut self, i: @mut I) {
        sorted_insert(&mut self.integrators, i as @mut Integrator<O>, |a, b| a.priority() < b.priority());

        for o in self.objects.mut_iter() {
            i.add(*o)
        }
    }

    pub fn add_solver<S: 'static + Solver<C>>(&mut self, s: @mut S) {
        sorted_insert(&mut self.solvers, s as @mut Solver<C>, |a, b| a.priority() < b.priority());
    }

    pub fn add_object(&mut self, b: @mut O) {
        self.objects.push(b);

        let b = self.objects.last();

        for d in self.integrators.mut_iter() {
            d.add(*b)
        }

        for d in self.detectors.mut_iter() {
            d.add(*b)
        }
    }

    pub fn remove_object(&mut self, b: @mut O) {
        match self.objects.iter().position(|o| managed::mut_ptr_eq(b, *o)) {
            Some(i) => {
                self.objects.swap_remove(i);

                for d in self.integrators.mut_iter() {
                    d.remove(b)
                }

                for d in self.detectors.mut_iter() {
                    d.remove(b)
                }
            },
            None => { }
        }
    }

    pub fn objects<'r>(&'r self) -> &'r [@mut O] {
        let res: &'r [@mut O] = self.objects;

        res
    }

    pub fn integrators<'r>(&'r self) -> &'r [@mut Integrator<O>] {
        let res: &'r [@mut Integrator<O>] = self.integrators;

        res
    }

    pub fn detectors<'r>(&'r self) -> &'r [@mut Detector<O, C>] {
        let res: &'r [@mut Detector<O, C>] = self.detectors;

        res
    }

    pub fn solvers<'r>(&'r self) -> &'r [@mut Solver<C>] {
        let res: &'r [@mut Solver<C>] = self.solvers;

        res
    }
}

#[inline(always)]
fn sorted_insert<T>(vec: &mut ~[T], t: T, lt: |&T, &T| -> bool) {
    let mut iinsert = vec.len();

    for (i, e) in vec.iter().enumerate() {
        if lt(&t, e) {
            iinsert = i;
            break;
        }
    }

    vec.insert(iinsert, t)
}
