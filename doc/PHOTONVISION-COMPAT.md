# Temporary PhotonVision alpha-2 compatibility bridge

PhotonVision `v2027.0.0-alpha-2` still links to WPILib APIs removed in the 2027 alpha7/alpha8 libraries. The temporary types under `src/main/java/org/wpilib/vision/apriltag/` wrap the new `org.wpilib.fields.Field` for PhotonVision's old `AprilTagFieldLayout`, `AprilTag`, and `AprilTagFields` references. `src/main/java/org/wpilib/math/util/Pair.java` supplies PhotonVision's old binary name; RBSI source uses `org.wpilib.util.Pair` directly.

The layout bridge reads the seven existing deploy JSON files, which use the old `tags`/`field` schema, and constructs a new `Field` with the same tag poses and dimensions. It is limited to the methods referenced by this project and PhotonVision alpha-2.

**Remove after a compatible PhotonVision release is installed:** delete the four compatibility types, migrate `FieldConstants` and callers to `org.wpilib.fields.Field`, and convert or replace the deploy layout JSON files if the new `Field.loadFromFile` schema is used. Verify `VisionSystemSim`, `PhotonCameraSim`, and the full test suite before deleting this note.
