//! Volume and inertia tensor computation.

#[doc(inline)]
pub use self::volumetric::{InertiaTensor, Volumetric};

pub use self::volumetric_ball::{ball_area, ball_center_of_mass, ball_unit_angular_inertia,
                                ball_volume};
pub use self::volumetric_cuboid::{cuboid_area, cuboid_center_of_mass, cuboid_unit_angular_inertia,
                                  cuboid_volume};
#[cfg(feature = "dim2")]
pub use self::volumetric_convex2::{convex_hull_area, convex_hull_center_of_mass,
                                   convex_hull_unit_angular_inertia, convex_hull_volume,
                                   convex_polyline_area_and_center_of_mass_unchecked,
                                   convex_polyline_area_unchecked,
                                   convex_polyline_mass_properties_unchecked};
// #[cfg(feature = "dim3")]
// pub use self::volumetric_convex3::{convex_hull_area, convex_hull_center_of_mass,
//                                    convex_hull_unit_angular_inertia, convex_hull_volume,
//                                    convex_mesh_area_unchecked,
//                                    convex_mesh_mass_properties_unchecked,
//                                    convex_mesh_volume_and_center_of_mass_unchecked};

#[doc(hidden)]
pub mod volumetric;

mod volumetric_ball;
mod volumetric_cuboid;
#[cfg(feature = "dim2")]
mod volumetric_convex2;
// #[cfg(feature = "dim3")]
// mod volumetric_convex3;
mod volumetric_compound;
mod volumetric_shape;
