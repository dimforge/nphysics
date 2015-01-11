//! Volume and inertia tensor computation.

#[doc(inline)]
pub use volumetric::volumetric::{Volumetric, InertiaTensor};

pub use volumetric::volumetric_ball::{ball_volume, ball_surface, ball_center_of_mass,
                                      ball_unit_angular_inertia};
pub use volumetric::volumetric_cylinder::{cylinder_volume, cylinder_surface,
                                          cylinder_center_of_mass, cylinder_unit_angular_inertia};
pub use volumetric::volumetric_cone::{cone_volume, cone_surface,
                                      cone_center_of_mass, cone_unit_angular_inertia};
pub use volumetric::volumetric_cuboid::{cuboid_volume, cuboid_surface,
                                        cuboid_center_of_mass, cuboid_unit_angular_inertia};
pub use volumetric::volumetric_convex::{convex_mesh_surface, convex_mesh_volume_and_center_of_mass,
                                        convex_mesh_mass_properties, convex_hull_surface,
                                        convex_hull_volume, convex_hull_center_of_mass,
                                        convex_hull_unit_angular_inertia};

#[doc(hidden)]
pub mod volumetric;

mod volumetric_ball;
mod volumetric_cylinder;
mod volumetric_cuboid;
mod volumetric_cone;
mod volumetric_compound;
mod volumetric_convex;
mod volumetric_repr;
