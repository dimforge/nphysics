//! Volume and inertia tensor computation.

#[doc(inline)]
pub use self::volumetric::{Volumetric, InertiaTensor};

pub use self::volumetric_ball::{ball_volume, ball_area, ball_center_of_mass,
                                ball_unit_angular_inertia};
pub use self::volumetric_cylinder::{cylinder_volume, cylinder_area,
                                    cylinder_center_of_mass, cylinder_unit_angular_inertia};
pub use self::volumetric_cone::{cone_volume, cone_area,
                                cone_center_of_mass, cone_unit_angular_inertia};
pub use self::volumetric_cuboid::{cuboid_volume, cuboid_area,
                                  cuboid_center_of_mass, cuboid_unit_angular_inertia};
#[cfg(feature = "dim2")]
pub use self::volumetric_convex2::{convex_polyline_area_unchecked,
                                   convex_polyline_area_and_center_of_mass_unchecked,
                                   convex_polyline_mass_properties_unchecked,
                                   convex_hull_area, convex_hull_volume, convex_hull_center_of_mass,
                                   convex_hull_unit_angular_inertia};
#[cfg(feature = "dim3")]
pub use self::volumetric_convex3::{convex_mesh_area_unchecked,
                                   convex_mesh_volume_and_center_of_mass_unchecked,
                                   convex_mesh_mass_properties_unchecked,
                                   convex_hull_area, convex_hull_volume, convex_hull_center_of_mass,
                                   convex_hull_unit_angular_inertia};

#[doc(hidden)]
pub mod volumetric;

mod volumetric_ball;
mod volumetric_cylinder;
mod volumetric_cone;
mod volumetric_cuboid;
#[cfg(feature = "dim2")]
mod volumetric_convex2;
#[cfg(feature = "dim3")]
mod volumetric_convex3;
mod volumetric_compound;
mod volumetric_shape;
