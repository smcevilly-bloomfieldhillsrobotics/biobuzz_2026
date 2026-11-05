package org.firstinspires.ftc.teamcode

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path

/**
 * Defines standard paths for the autonomous routine using Pedro Pathing 3.0 with PoseFactory.degrees().
 * Poses are specified in degrees directly without manual radian conversion:
 *
 * 1. start:                             (56.0, 8.0, 90.0°)
 * 2. startPath (Shooting Position):     (56.0, 36.0, 0.0°)
 * 3. reloadingStartPathStart:           (56.0, 36.0, 90.0°)
 * 4. reloadingStartPathControl1:        (56.0, 20.0, 0.0°)
 * 5. reloadingStartPath:                (34.8008, 10.97, 180.0°)
 * 6. loadGardenPathStart:               (34.8008, 10.97, 180.0°)
 * 7. loadGardenPathControl1:            (17.8, 12.58, 0.0°)
 * 8. loadGardenPath:                    (14.2165, 36.3113, -5.4183°)
 * 9. oppositeShootingPositionPathStart: (14.2165, 36.3113, -5.4183°)
 * 10. oppositeShootingPositionControl1: (8.5504, 83.5115, 0.0°)
 * 11. oppositeShootingPositionControl2: (42.9874, 88.2136, 0.0°)
 * 12. oppositeShootingPositionPath:     (55.9838, 132.8422, 270.0°)
 */
class PedroPaths {

    private val poseFactory = PoseFactory.degrees()

    val startPose: Pose = poseFactory.of(56.0, 8.0, 90.0)

    val shootingPositionPose: Pose = poseFactory.of(56.0, 36.0, 90.0)

    val reloadingStartPathStart: Pose = poseFactory.of(56.0, 36.0, 90.0)
    val reloadingStartPathControl1: Pose = poseFactory.of(56.0, 20.0, 90.0)
    val reloadingStartPose: Pose = poseFactory.of(34.8, 11.0, 90.0)

    val loadGardenPathStart: Pose = poseFactory.of(34.8, 11.0, 90.0)
    val loadGardenPathControl1: Pose = poseFactory.of(18.0, 12.6, 90.0)
    val loadGardenPose: Pose = poseFactory.of(11.0, 14.2, 90.0) // X=11.0 for 2" clearance from X=0 wall

    val oppositeShootingPositionPathStart: Pose = poseFactory.of(11.0, 14.2, 90.0)
    val oppositeShootingPositionPathControl1: Pose = poseFactory.of(18.0, 85.0, 90.0) // X=18.0 for 9" safe wall clearance
    val oppositeShootingPositionPose: Pose = poseFactory.of(52.8, 126.9, 270.0)

    // Trajectory functions configured for constant 90° heading throughout

    val shootingPositionPath: Path get() = Paths.line(startPose, shootingPositionPose).constant(Math.toRadians(90.0))

    val reloadingStartPath: Path get() = Paths.curve(reloadingStartPathStart, reloadingStartPathControl1, reloadingStartPose)
        .constant(Math.toRadians(90.0))

    val loadGardenPath: Path get() = Paths.curve(loadGardenPathStart, loadGardenPathControl1, loadGardenPose)
        .constant(Math.toRadians(90.0))

    val oppositeShootingPositionPath: Path get() = Paths.curve(
        oppositeShootingPositionPathStart,
        oppositeShootingPositionPathControl1,
        oppositeShootingPositionPose
    ).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    // AprilTag and Center Hive Auto Poses
    val centerHiveStartPose: Pose = poseFactory.of(72.0, 9.0, 90.0)
    val centerHiveIntermediatePose: Pose = poseFactory.of(72.0, 120.0, 90.0)
    val centerHiveFinalPose: Pose = poseFactory.of(53.0, 120.0, 270.0)

    val centerHiveIntermediatePath: Path get() = Paths.line(centerHiveStartPose, centerHiveIntermediatePose).constant(Math.toRadians(90.0))
    val centerHiveFinalPath: Path get() = Paths.line(centerHiveIntermediatePose, centerHiveFinalPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    val aprilTagStartPose: Pose = poseFactory.of(72.0, 9.0, 90.0)
    val aprilTagIntermediatePose: Pose = poseFactory.of(72.0, 125.0, 90.0)
    val aprilTagFinalPose: Pose = poseFactory.of(55.0, 125.0, 270.0)

    val aprilTagIntermediatePath: Path get() = Paths.line(aprilTagStartPose, aprilTagIntermediatePose).constant(Math.toRadians(90.0))
    val aprilTagFinalPath: Path get() = Paths.line(aprilTagIntermediatePose, aprilTagFinalPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    // Parking Path Waypoints: (60, 9) -> (24, 24) -> (24, 110) -> (15, 110)
    val parkStartPose: Pose = poseFactory.of(60.0, 9.0, 90.0)
    val parkPose1: Pose = poseFactory.of(24.0, 24.0, 90.0)
    val parkPose2: Pose = poseFactory.of(24.0, 110.0, 90.0)
    val parkEndPose: Pose = poseFactory.of(15.0, 110.0, 90.0)

    val parkPathSegment1: Path get() = Paths.line(parkStartPose, parkPose1).constant(Math.toRadians(90.0))
    val parkPathSegment2: Path get() = Paths.line(parkPose1, parkPose2).constant(Math.toRadians(90.0))
    val parkPathSegment3: Path get() = Paths.line(parkPose2, parkEndPose).constant(Math.toRadians(90.0))

    // Compound path covering the full parking trajectory in one command
    val parkPath: Path get() = Paths.path(parkPathSegment1, parkPathSegment2, parkPathSegment3)
}

