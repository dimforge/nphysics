use na::{self, RealField};

/// Parameters for a time-step of the physics engine.
#[derive(Clone)]
pub struct IntegrationParameters<N: RealField> {
    /// The timestep (default: `1.0 / 60.0`)
    dt: N,
    /// The inverse of `dt`.
    inv_dt: N,
    /// If `false`, CCD will be globally disabled independently from collider CCD status.
    ///
    /// Default: `true`.
    pub ccd_enabled: bool,
    /// If `true`, the world's `step` method will stop right after resolving exactly one CCD event.
    /// This allows the user to take action during a timestep, in-between two CCD events.
    ///
    /// Default: `false`.
    pub substepping_enabled: bool,
    /// The total elapsed time in the physics world.
    ///
    /// This is the accumulation of the `dt` of all the calls to `world.step()`.
    pub t: N,
    /// The Error Reduction Parameter in `[0, 1]` is the proportion of
    /// the positional error to be corrected at each time step (default: `0.2`).
    pub erp: N,
    /// Each cached impulse are multiplied by this coefficient in `[0, 1]`
    /// when they are re-used to initialize the solver (default `1.0`).
    pub warmstart_coeff: N,
    /// Contacts at points where the involved bodies have a relative
    /// velocity smaller than this threshold wont be affected by the restitution force (default: `1.0`).
    pub restitution_velocity_threshold: N,
    /// Ammount of penetration the engine wont attempt to correct (default: `0.001m`).
    pub allowed_linear_error: N,
    /// Ammount of angular drift of joint limits the engine wont
    /// attempt to correct (default: `0.001rad`).
    pub allowed_angular_error: N,
    /// Maximum linear correction during one step of the non-linear position solver (default: `100.0`).
    pub max_linear_correction: N,
    /// Maximum angular correction during one step of the non-linear position solver (default: `0.2`).
    pub max_angular_correction: N,
    /// Maximum nonlinear SOR-prox scaling parameter when the constraint
    /// correction direction is close to the kernel of the involved multibody's
    /// jacobian (default: `0.2`).
    pub max_stabilization_multiplier: N,
    /// Maximum number of iterations performed by the velocity constraints solver.
    pub max_velocity_iterations: usize,
    /// Maximum number of iterations performed by the position-based constraints solver.
    pub max_position_iterations: usize,
}

impl<N: RealField> IntegrationParameters<N> {
    /// Creates a set of integration parameters with the given values.
    pub fn new(
        dt: N,
        erp: N,
        warmstart_coeff: N,
        restitution_velocity_threshold: N,
        allowed_linear_error: N,
        allowed_angular_error: N,
        max_linear_correction: N,
        max_angular_correction: N,
        max_stabilization_multiplier: N,
        max_velocity_iterations: usize,
        max_position_iterations: usize,
        ccd_enabled: bool,
        substepping_enabled: bool,
    ) -> Self {
        IntegrationParameters {
            t: N::zero(),
            dt,
            inv_dt: if dt == N::zero() {
                N::zero()
            } else {
                N::one() / dt
            },
            erp,
            warmstart_coeff,
            restitution_velocity_threshold,
            allowed_linear_error,
            allowed_angular_error,
            max_linear_correction,
            max_angular_correction,
            max_stabilization_multiplier,
            max_velocity_iterations,
            max_position_iterations,
            ccd_enabled,
            substepping_enabled,
        }
    }

    /// The current time-stepping length.
    #[inline(always)]
    pub fn dt(&self) -> N {
        self.dt
    }

    /// The inverse of the time-stepping length.
    ///
    /// This is zero if `self.dt` is zero.
    #[inline(always)]
    pub fn inv_dt(&self) -> N {
        self.inv_dt
    }

    /// Sets the time-stepping length.
    ///
    /// This automatically recompute `self.inv_dt`.
    #[inline]
    pub fn set_dt(&mut self, dt: N) {
        assert!(dt >= N::zero(), "The time-stepping length cannot be negative.");
        self.dt = dt;
        if dt == N::zero() {
            self.inv_dt = N::zero()
        } else {
            self.inv_dt = N::one() / dt
        }
    }

    /// Sets the inverse time-stepping length (i.e. the frequency).
    ///
    /// This automatically recompute `self.dt`.
    #[inline]
    pub fn set_inv_dt(&mut self, inv_dt: N) {
        self.inv_dt = inv_dt;
        if inv_dt == N::zero() {
            self.dt = N::zero()
        } else {
            self.dt = N::one() / inv_dt
        }
    }
}

impl<N: RealField> Default for IntegrationParameters<N> {
    fn default() -> Self {
        Self::new(
            na::convert(1.0 / 60.0),
            na::convert(0.2),
            na::convert(1.0),
            na::convert(1.0),
            na::convert(0.001),
            na::convert(0.001),
            na::convert(0.2), // 100000.0),
            na::convert(0.2),
            na::convert(0.2),
            8,
            3,
            true,
            false,
        )
    }
}
