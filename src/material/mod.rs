pub use self::material::{Material, MaterialContext, MaterialCombineMode, MaterialHandle, MaterialId, LocalMaterialProperties};
pub use self::basic_material::BasicMaterial;
pub use self::materials_coefficients_table::MaterialsCoefficientsTable;

mod material;
mod basic_material;
mod materials_coefficients_table;