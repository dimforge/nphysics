# Change Log
All notable changes to `nalgebra`, starting with the version 0.6.0 will be
documented here.

This project adheres to [Semantic Versioning](http://semver.org/).

## [0.4.0]
### Modified
  * Use the latest **ncollide** API v0.9.0 which included breaking changes.
  * Rename event handler registration methods and traits to remove
    `signal_` and `Signal` from their names.

## [0.3.0]
### Added
  * Added sensors.

### Modified
  * `World::add_body` and `World::remove_body` have been renamed to
    `World::add_rigid_body` and `World::remove_rigid_body`.
  * `World::add_ccd_to(...)` has an additional argument indicating whether
    rigid bodies with CCD should trigger events for sensors it should have
    traversed.
  * `World::interferences()` has been renamed to `World::constraints()`.
  * `Volumetric::surface()` has been renamed to `Volumetric::area()`.
