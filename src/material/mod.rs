//! Material data structures.

pub use self::basic_material::BasicMaterial;
pub use self::material::{
    LocalMaterialProperties, Material, MaterialCombineMode, MaterialContext, MaterialHandle,
    MaterialId,
};
pub use self::materials_coefficients_table::MaterialsCoefficientsTable;

mod basic_material;
mod material;
mod materials_coefficients_table;
