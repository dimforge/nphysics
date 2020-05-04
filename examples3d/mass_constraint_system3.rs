extern crate nalgebra as na;

use kiss3d::loader::obj;
use na::{Isometry3, Point3, RealField, Vector3};
use ncollide3d::procedural;
use ncollide3d::shape::{Cuboid, ShapeHandle, TriMesh};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, DeformableColliderDesc,
    Ground, MassConstraintSystemDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;
use std::f32;
use std::path::Path;

pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(r!(0.0), r!(-9.81), r!(0.0)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    let ground_thickness = r!(0.2);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        r!(50.0),
        ground_thickness,
        r!(50.0),
    )));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the deformable body and a collider for its contour.
     */
    let obj_path = "media/models/rust_logo_simplified.obj";
    let obj = obj::parse_file(&Path::new(&obj_path), &Path::new(""), "");

    if let Ok(model) = obj {
        let mut meshes: Vec<procedural::TriMesh<N>> = model
            .into_iter()
            .map(|mesh| {
                let mesh = mesh.1.to_trimesh().unwrap();
                procedural::TriMesh {
                    coords: mesh
                        .coords
                        .into_iter()
                        .map(na::convert::<_, Point3<f64>>)
                        .map(na::convert::<_, Point3<N>>)
                        .collect(),
                    normals: mesh.normals.map(|n| {
                        n.into_iter()
                            .map(na::convert::<_, Vector3<f64>>)
                            .map(na::convert::<_, Vector3<N>>)
                            .collect()
                    }),
                    uvs: None, // We don't care about uvs.
                    indices: mesh.indices,
                }
            })
            .collect();
        meshes[0].split_index_buffer(true);

        let rot = Vector3::x() * r!(std::f64::consts::FRAC_PI_2);
        let trimesh1 = TriMesh::from(meshes[0].clone())
            .scaled(&Vector3::repeat(r!(0.5)))
            .transformed(&Isometry3::new(Vector3::y() * r!(5.0), rot));
        let trimesh2 = TriMesh::from(meshes[0].clone())
            .scaled(&Vector3::repeat(r!(0.5)))
            .transformed(&Isometry3::new(Vector3::y() * r!(9.5), rot));

        let mut deformable1 = MassConstraintSystemDesc::from_trimesh(&trimesh1)
            .stiffness(Some(r!(0.1)))
            .build();
        deformable1.generate_neighbor_constraints(Some(r!(0.1)));
        deformable1.generate_neighbor_constraints(Some(r!(0.1)));
        let deformable1_handle = bodies.insert(deformable1);
        let co1 = DeformableColliderDesc::new(ShapeHandle::new(trimesh1)).build(deformable1_handle);
        colliders.insert(co1);

        let mut deformable2 = MassConstraintSystemDesc::from_trimesh(&trimesh2)
            .set_stiffness(Some(r!(100.0)))
            .build();
        deformable2.generate_neighbor_constraints(Some(r!(100.0)));
        deformable2.generate_neighbor_constraints(Some(r!(100.0)));
        let deformable2_handle = bodies.insert(deformable2);
        let co2 = DeformableColliderDesc::new(ShapeHandle::new(trimesh2)).build(deformable2_handle);
        colliders.insert(co2);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point3::new(10.0, 4.0, 10.0), Point3::new(0.0, 4.0, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Mass-constraint system", init_world)]);
    testbed.run()
}
