use na::Real;
use na;
use ncollide::shape::Ball;
use volumetric::Volumetric;
use math::{AngularInertia, Point, DIM};

/// The volume of a ball.
#[inline]
pub fn ball_volume<N: Real>(radius: N) -> N {
    if DIM == 2 {
        let _pi = N::pi();
        _pi * radius * radius
    } else {
        let _pi = N::pi();
        _pi * radius * radius * radius * na::convert(4.0f64 / 3.0)
    }
}

/// The area of a ball.
#[inline]
pub fn ball_area<N: Real>(radius: N) -> N {
    if DIM == 2 {
        let _pi = N::pi();
        _pi * radius * na::convert(2.0f64)
    } else {
        let _pi = N::pi();
        _pi * radius * radius * na::convert(4.0f64)
    }
}

/// The center of mass of a ball.
#[inline]
pub fn ball_center_of_mass<N: Real>() -> Point<N> {
    Point::origin()
}

/// The unit angular inertia of a ball.
#[inline]
pub fn ball_unit_angular_inertia<N: Real>(radius: N) -> AngularInertia<N> {
    let diag;
    if DIM == 2 {
        diag = radius * radius / na::convert(2.0f64);
    } else {
        diag = radius * radius * na::convert(2.0f64 / 5.0);
    }

    AngularInertia::from_diagonal_element(diag)
}

impl<N: Real> Volumetric<N> for Ball<N> {
    fn area(&self) -> N {
        ball_area(self.radius())
    }

    fn volume(&self) -> N {
        ball_volume(self.radius())
    }

    fn center_of_mass(&self) -> Point<N> {
        ball_center_of_mass()
    }

    fn unit_angular_inertia(&self) -> AngularInertia<N> {
        ball_unit_angular_inertia(self.radius())
    }
}
