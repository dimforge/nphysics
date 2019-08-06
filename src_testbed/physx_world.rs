use std::collections::HashMap;

use std::f32;
use std::ptr;
use na::{Isometry3, Vector3};
use ncollide::shape;
use nphysics::counters::Counters;
use nphysics::object::{ColliderAnchor, DefaultBodySet, DefaultColliderSet, DefaultBodyHandle, Multibody};
use nphysics::joint::{DefaultJointConstraintSet, RevoluteJoint, PrismaticJoint, BallJoint};
use nphysics::force_generator::DefaultForceGeneratorSet;
use nphysics::world::DefaultMechanicalWorld;

use physx::prelude::*;
use physx_sys::*;

const PX_PHYSICS_VERSION: u32 = physx::version(4, 1, 0);

pub struct PhysXWorld {
    foundation: Foundation,
    cooking: Cooking,
    physics: Physics,
    scene: Box<Scene>,
    nphysics2physx: HashMap<DefaultBodyHandle, BodyHandle>,
}

unsafe extern "C" fn filter_shader(infos: *mut FilterShaderCallbackInfo) -> u16
{
    (*(*infos).pairFlags).mBits = PxPairFlag::eSOLVE_CONTACT as u16;
    (*(*infos).pairFlags).mBits |= PxPairFlag::eDETECT_DISCRETE_CONTACT as u16;
    (*(*infos).pairFlags).mBits |= PxPairFlag::eDETECT_CCD_CONTACT as u16;
    PxFilterFlag::eDEFAULT as u16
}


impl PhysXWorld {
    pub fn from_nphysics(
        mechanical_world: &DefaultMechanicalWorld<f32>,
        bodies: &DefaultBodySet<f32>,
        colliders: &DefaultColliderSet<f32>,
        _joint_constraints: &DefaultJointConstraintSet<f32>,
        _force_generators: &DefaultForceGeneratorSet<f32>
    )
    -> Self {
        let mut foundation = Foundation::new(PX_PHYSICS_VERSION);

        let tolerance_scale = unsafe { PxTolerancesScale_new() };
        let cooking = Cooking::new(PX_PHYSICS_VERSION, &mut foundation, unsafe { PxCookingParams_new(&tolerance_scale) });
        let mut physics = PhysicsBuilder::default()
            .load_extensions(false)
            .build(&mut foundation);

        // Create the scene.
        // We don't use physics.create_scene because it does not give us the choice to enable TGS.
        let scene_desc = unsafe {
            let tolerances = physics.get_tolerances_scale();
            let mut scene_desc = PxSceneDesc_new(tolerances);

            scene_desc.cpuDispatcher = phys_PxDefaultCpuDispatcherCreate(1, ptr::null_mut()) as *mut _;
            scene_desc.gravity = physx::transform::na_to_px_v3(mechanical_world.gravity);
//            scene_desc.filterShader = filter_shader as *mut std::ffi::c_void; // physx_sys::get_default_simulation_filter_shader();
            scene_desc.solverType = PxSolverType::eTGS;
            scene_desc.flags.mBits |= PxSceneFlag::eENABLE_CCD;

            physx_sys::enable_custom_filter_shader(
                &mut scene_desc,
                filter_shader,
            );


            scene_desc
        };

        let scene = unsafe { PxPhysics_createScene_mut(physics.get_raw_mut(), &scene_desc) };
        let mut scene = Box::new(Scene::new(scene));
        let scene_ptr = scene.as_mut() as *mut Scene;
        scene.set_simulation_event_callback(physx::physics::on_contact_callback, scene_ptr);

        /*

        let scene = physics.create_scene(
            SceneBuilder::default()
                .set_gravity(mechanical_world.gravity)
                .set_simulation_threading(SimulationThreadType::Dedicated(1)),
        );
        */

        let mut res = Self {
            foundation,
            cooking,
            physics,
            scene,
            nphysics2physx: HashMap::new(),
        };

        res.insert_bodies(bodies, colliders);

        res
    }

    fn insert_bodies(&mut self, bodies: &DefaultBodySet<f32>, colliders: &DefaultColliderSet<f32>) {
        unsafe {
            let material = self.physics.create_material(0.5, 0.5, 0.0);

            // Setup bodies.
            for (handle, body) in bodies.iter() {
                if let Some(mb) = body.downcast_ref::<Multibody<f32>>() {
                    use physx::articulation_link::ArticulationLinkBuilder;
                    use physx::articulation_reduced_coordinate::ArticulationFlag;

                    let root_link = ArticulationLinkBuilder::default();
                    let mut articulation = ArticulationReducedCoordinate::new_with_link(&root_link, &mut self.physics, &mut self.cooking);
                    articulation.set_articulation_flag(ArticulationFlag::FixBase, true);
                    let mut parent = articulation.root_handle();

                    for (i, mb_link) in mb.links().enumerate() {
                        let link = ArticulationLinkBuilder::default()
                            .build(&mut articulation, parent, None);


                        let link_mut = articulation.part_mut(i + 1).unwrap();
                        let joint = PxArticulationLink_getInboundJoint(link_mut.get_raw());

                        let parent_shift = Isometry3::new(*mb_link.parent_shift(), na::zero());
                        let body_shift = Isometry3::new(*mb_link.body_shift(), na::zero());
                        let parent_pose = physx::transform::na_to_px_tf_ref(&parent_shift.to_homogeneous());
                        let child_pose = physx::transform::na_to_px_tf_ref(&body_shift.to_homogeneous());
                        PxArticulationJointBase_setParentPose_mut(joint, &parent_pose);
                        PxArticulationJointBase_setChildPose_mut(joint, &child_pose);

                        if let Some(_revolute) = mb_link.joint().downcast_ref::<RevoluteJoint<f32>>() {
                            PxArticulationJointReducedCoordinate_setJointType_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationJointType::eREVOLUTE);
                            PxArticulationJointReducedCoordinate_setMotion_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
                        }

                        if let Some(_ball) = mb_link.joint().downcast_ref::<BallJoint<f32>>() {
                            PxArticulationJointReducedCoordinate_setJointType_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationJointType::eSPHERICAL);
                            PxArticulationJointReducedCoordinate_setMotion_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
                            PxArticulationJointReducedCoordinate_setMotion_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
                            PxArticulationJointReducedCoordinate_setMotion_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);

                        }

                        if let Some(prismatic) = mb_link.joint().downcast_ref::<PrismaticJoint<f32>>() {
                            PxArticulationJointReducedCoordinate_setJointType_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationJointType::ePRISMATIC);

                            if prismatic.min_offset().is_some() || prismatic.max_offset().is_some() {
                                PxArticulationJointReducedCoordinate_setMotion_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationAxis::eY, PxArticulationMotion::eLIMITED);
                                let min_offset = prismatic.min_offset().unwrap_or(f32::MIN);
                                let max_offset = prismatic.max_offset().unwrap_or(f32::MAX);
                                PxArticulationJointReducedCoordinate_setLimit_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationAxis::eY, min_offset, max_offset);
                            } else {
                                PxArticulationJointReducedCoordinate_setMotion_mut(joint as *mut PxArticulationJointReducedCoordinate, PxArticulationAxis::eY, PxArticulationMotion::eFREE);
                            }
                        }

                        parent = link;
                    }

                    let physx_handle = self.scene.add_articulation(articulation, |_| {});
                    self.nphysics2physx.insert(handle, physx_handle.into());
                } else {
                    // This handle both the Ground and RigidBody cases.
                    let part = body.part(0).unwrap();
                    let pose = part.position();
                    let pose = physx::transform::na_to_px_tf_ref(&pose.to_homogeneous());
                    let physx_handle;

                    if body.is_static() {
                        let rb = PxPhysics_createRigidStatic_mut(self.physics.get_raw_mut(), &pose);
                        physx_handle = self.scene.add_actor(physx::rigid_static::RigidStatic::new(rb));
                    } else {
                        let vel = part.velocity();
                        let linvel = physx::transform::na_to_px_v3(vel.linear);
                        let angvel = physx::transform::na_to_px_v3(vel.angular);
                        let rb = PxPhysics_createRigidDynamic_mut(self.physics.get_raw_mut(), &pose);
                        PxRigidDynamic_setSolverIterationCounts_mut(rb, 3, 8);
                        PxRigidDynamic_setSleepThreshold_mut(rb, 0.0);
                        PxRigidBody_setLinearVelocity_mut(rb as *mut physx_sys::PxRigidBody, &linvel, false);
                        PxRigidBody_setAngularVelocity_mut(rb as *mut physx_sys::PxRigidBody, &angvel, false);

                        physx_handle = self.scene.add_dynamic(physx::rigid_dynamic::RigidDynamic::new(rb));
                    }

                    self.nphysics2physx.insert(handle, physx_handle);
                }
            }

            // Setup colliders.
            let mut densities = HashMap::new();
            for (_, collider) in colliders.iter() {
                match collider.anchor() {
                    ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                        let physx_body_handle = self.nphysics2physx[&body_part.0];
                        let geometries = Self::physx_geometry_from_ncollide_shape(&mut self.cooking, collider.shape());

                        for (geometry, delta) in &geometries {
                            let shape_flags = if collider.is_sensor() {
                                PxShapeFlag::eTRIGGER_SHAPE as u8
                            } else {
                                PxShapeFlag::eSIMULATION_SHAPE as u8
                            };
                            let shape = PxPhysics_createShape_mut(self.physics.get_raw_mut(), geometry.as_raw(), material, true, PxShapeFlags { mBits: shape_flags });
                            let delta = physx::transform::na_to_px_tf_ref(&(position_wrt_body_part * delta).to_homogeneous());
                            PxShape_setLocalPose_mut(shape, &delta);
                            physx_sys::PxShape_setRestOffset_mut(shape, collider.query_type().query_limit());

                            let actor = if let Some(multibody) = self.scene.get_multibody_mut(physx_body_handle) {
                                multibody.part_mut(body_part.1 + 1).unwrap().get_raw_mut() as *mut PxRigidActor
                            } else {
                                let actor = self.scene.get_rigid_actor(physx_body_handle).unwrap();
                                let raw_actor = actor.get_raw();
                                self.scene.find_matching_rigid_actor_mut(raw_actor).unwrap().get_raw_mut()
                            };

                            PxRigidActor_attachShape_mut(actor, shape);

                            if collider.is_ccd_enabled() {
                                PxRigidBody_setRigidBodyFlag_mut(actor as *mut PxRigidBody, PxRigidBodyFlag::eENABLE_CCD, true);
                            }

                            densities
                                .entry(body_part)
                                .or_insert(Vec::new())
                                .push(collider.density());
                        }
                    }
                    ColliderAnchor::OnDeformableBody { .. } => {
                        eprintln!("Deformable bodies are not supported by the PhysX backend.")
                    }
                }
            }

            // Recompute masses and inertia.
            for (handle, densities) in &densities {
                let physx_body_handle = self.nphysics2physx[&handle.0];

                let actor = if let Some(multibody) = self.scene.get_multibody_mut(physx_body_handle) {
                    Some(multibody.part_mut(handle.1 + 1).unwrap().get_raw_mut() as *mut PxRigidBody)
                } else {
                    self.scene.get_dynamic_mut(physx_body_handle).map(|a| a.get_raw_mut() as *mut PxRigidBody)
                };

                if let Some(actor) = actor {
                    PxRigidBodyExt_updateMassAndInertia_mut(
                        actor,
                        densities.as_ptr(),
                        densities.len() as u32,
                        ptr::null(),
                        false
                    );
                }
            }
        }
    }

    fn physx_geometry_from_ncollide_shape(cooking: &mut Cooking, shape: &shape::Shape<f32>) -> Vec<(PhysicsGeometry, Isometry3<f32>)> {
        let mut result = Vec::new();

        if let Some(_s) = shape.as_shape::<shape::Plane<f32>>() {
        } else if let Some(s) = shape.as_shape::<shape::Ball<f32>>() {
            let geom = PhysicsGeometry::from(&ColliderDesc::Sphere(s.radius()));
            result.push((geom, Isometry3::identity()))
        } else if let Some(s) = shape.as_shape::<shape::Cuboid<f32>>() {
            let half_extents = s.half_extents();
            let geom = PhysicsGeometry::from(&ColliderDesc::Box(half_extents.x, half_extents.y, half_extents.z));
            result.push((geom, Isometry3::identity()))
        } else if let Some(s) = shape.as_shape::<shape::Capsule<f32>>() {
            let geom = PhysicsGeometry::from(&ColliderDesc::Capsule(s.radius(), s.height()));
            result.push((geom, Isometry3::rotation(Vector3::z() * f32::consts::PI / 2.0)))
        } else if let Some(cp) = shape.as_shape::<shape::Compound<f32>>() {
            result = cp.shapes()
                .iter()
                .flat_map(|(shift, shape)| {
                    Self::physx_geometry_from_ncollide_shape(cooking, &**shape)
                        .into_iter()
                        .map(move |(geom, delta)| (geom, shift * delta))
                })
                .collect();
        } else if let Some(ch) = shape.as_shape::<shape::ConvexHull<f32>>() {
            unsafe {
                let mut convex_desc = PxConvexMeshDesc_new();
                convex_desc.points.count = ch.points().len() as u32;
                convex_desc.points.stride = (3 * std::mem::size_of::<f32>()) as u32;
                convex_desc.points.data = ch.points().as_ptr() as *const std::ffi::c_void;
                if ch.points().len() < 8 {
                    // The plane shifting algorithm must be used if there are less than 8 points.
                    convex_desc.flags = PxConvexFlags { mBits: (physx_sys::PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::ePLANE_SHIFTING) as u16 };
                } else {
                    convex_desc.flags = PxConvexFlags { mBits: physx_sys::PxConvexFlag::eCOMPUTE_CONVEX as u16 };
                }
                convex_desc.vertexLimit = ch.points().len() as u16;

                if PxConvexMeshDesc_isValid(&convex_desc) {
                    let insertion_callback =
                        PxPhysics_getPhysicsInsertionCallback_mut(phys_PxGetPhysics());

                    let convex = PxCooking_createConvexMesh(
                        cooking.get_raw_mut(),
                        &convex_desc,
                        insertion_callback,
                        ptr::null_mut(),
                    );

                    assert!(!convex.is_null());


                    let flags = PxConvexMeshGeometryFlags { mBits: 0 };
                    let scale = PxMeshScale_new();
                    let convex_geometry = PxConvexMeshGeometry_new_1(convex, &scale, flags);
                    let phys_geom = PhysicsGeometry::new(Geometry::ConvexMesh(convex_geometry));
                    result.push((phys_geom, Isometry3::identity()))
                }
            }
        } else if let Some(_s) = shape.as_shape::<shape::TriMesh<f32>>() {
        } else if let Some(_s) = shape.as_shape::<shape::HeightField<f32>>() {
        }

        result
    }

    pub fn step(&mut self, counters: &mut Counters, timestep: f32) {
        counters.step_started();
        self.scene.simulate(timestep);
        self.scene
            .fetch_results(true)
            .expect("error occurred during simulation");
        counters.step_completed();
    }

    pub fn sync(&self,
                _bodies: &mut DefaultBodySet<f32>,
                colliders: &mut DefaultColliderSet<f32>) {
        for (_, collider) in colliders.iter_mut() {
            match collider.anchor() {
                ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                    if let Some(px_handle) = self.nphysics2physx.get(&body_part.0) {
                        if let Some(multibody) = self.scene.get_multibody(*px_handle) {
                            let link = multibody.part(body_part.1 + 1).unwrap();
                            let pos: Isometry3<f32> = unsafe { na::convert_unchecked(link.get_global_pose()) };
                            let new_pos = pos * position_wrt_body_part;
                            collider.set_position(new_pos)
                        } else if let Some(actor) = self.scene.get_rigid_actor(*px_handle) {
                            let pos: Isometry3<f32> = unsafe { na::convert_unchecked(actor.get_global_pose()) };
                            let new_pos = pos * position_wrt_body_part;
                            collider.set_position(new_pos)
                        }
                    }
                }
                ColliderAnchor::OnDeformableBody { .. } => {}
            }
        }
    }
}


impl Drop for PhysXWorld {
    fn drop(&mut self) {
        unsafe {
            self.scene.release();
            self.physics.release();
            self.cooking.release();
            self.foundation.release();
        }
    }
}