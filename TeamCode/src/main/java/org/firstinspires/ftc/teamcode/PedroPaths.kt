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

    val aprilTagStartPose: Pose = poseFactory.of(63.0, 9.0, 90.0)
    val aprilTagIntermediatePose: Pose = poseFactory.of(63.0, 125.0, 90.0)
    val aprilTagFinalPose: Pose = poseFactory.of(55.0, 125.0, 270.0)

    // Aimed Poses: Centered toward the Hive Cell Target
    // Red Start Aim (63, 9) aiming at Red Audience Cell (59.7, 52.5) -> atan2(52.5 - 9.0, 59.7 - 63.0) = atan2(43.5, -3.3) = 94.3°
    val redStartAimPose: Pose = poseFactory.of(63.0, 9.0, 94.3)
    val redStartAimPath: Path get() = Paths.line(aprilTagStartPose, redStartAimPose).linear(Math.toRadians(90.0), Math.toRadians(94.3))
    val redStartRealignPath: Path get() = Paths.line(redStartAimPose, aprilTagStartPose).linear(Math.toRadians(94.3), Math.toRadians(90.0))

    // Red Opposite Aim (55, 125) aiming at Red Scoring Cell (59.7, 91.5) -> atan2(91.5 - 125.0, 59.7 - 55.0) = atan2(-33.5, 4.7) = 278.0°
    val redOppositeAimPose: Pose = poseFactory.of(55.0, 125.0, 278.0)
    val redOppositeAimPath: Path get() = Paths.line(aprilTagFinalPose, redOppositeAimPose).linear(Math.toRadians(270.0), Math.toRadians(278.0))
    val redOppositeRealignPath: Path get() = Paths.line(redOppositeAimPose, aprilTagFinalPose).linear(Math.toRadians(278.0), Math.toRadians(270.0))

    // Intermediate north motion using a curved Bezier trajectory with control point to smooth acceleration
    val redNorthControl1: Pose = poseFactory.of(63.0, 67.0, 90.0)
    val aprilTagIntermediatePath: Path get() = Paths.curve(aprilTagStartPose, redNorthControl1, aprilTagIntermediatePose).constant(Math.toRadians(90.0))
    
    // S-curve turn and strafe into opposite shooting position
    val redOppositeCurveControl: Pose = poseFactory.of(59.0, 125.0, 180.0)
    val aprilTagFinalPath: Path get() = Paths.curve(aprilTagIntermediatePose, redOppositeCurveControl, aprilTagFinalPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    // Parking Path Waypoints: (63, 9) -> (24, 24) -> (24, 105) -> (13, 105)
    // Red park tile is 2nd from top: Row 5 (Y: 96.0" to 120.0"), Left Column 1 (X: 0.0" to 24.0")
    // Left half of tile: X: 0.0" to 12.0". Bottom half: Y: 96.0" to 108.0"
    // Robot center at X=13.0, Y=105.0 gives 4.0" clearance to left wall (X=0) and 9.0" clearance to Row 4 boundary (Y=96)
    val parkStartPose: Pose = poseFactory.of(63.0, 9.0, 90.0)
    val parkPose1: Pose = poseFactory.of(24.0, 24.0, 90.0)
    val parkPose2: Pose = poseFactory.of(24.0, 105.0, 90.0)
    val parkEndPose: Pose = poseFactory.of(13.0, 105.0, 90.0)

    val parkPathSegment1: Path get() = Paths.line(parkStartPose, parkPose1).constant(Math.toRadians(90.0))
    val parkPathSegment2: Path get() = Paths.line(parkPose1, parkPose2).constant(Math.toRadians(90.0))
    val parkPathSegment3: Path get() = Paths.line(parkPose2, parkEndPose).constant(Math.toRadians(90.0))

    // Compound path covering the full parking trajectory in one command
    val parkPath: Path get() = Paths.path(parkPathSegment1, parkPathSegment2, parkPathSegment3)

    // Bezier control points to curve around the center hive/submersible (Tiles Col 3-4, Row 3-4: X=48 to 96, Y=48 to 96)
    // The metal bars on the left border of the center structure run along the X=48 line (Y: 48 to 96).
    // An 18" robot has half-width 9.0", so robot center X MUST be <= 38.0" to stay completely clear of the metal bars.
    // Setting Control 1 at (24.0, 24.0) and Control 2 at (18.0, 75.0) guarantees the robot center stays well below X=24.0,
    // giving more than 15 inches of buffer away from the metal bars at X=48, and 9.0" of clearance off the left wall (X=0).
    val startToParkControl1: Pose = poseFactory.of(24.0, 24.0, 90.0)
    val startToParkControl2: Pose = poseFactory.of(18.0, 75.0, 90.0)

    // Bezier control points from opposite launch (55, 125) down to park (13, 105)
    val redOppositeToParkControl1: Pose = poseFactory.of(28.0, 120.0, 270.0)
    val redOppositeToParkControl2: Pose = poseFactory.of(18.0, 112.0, 270.0)

    // Curve path from start directly to park avoiding the center hive and submersible bars
    val startToParkCurvePath: Path get() = Paths.curve(
        aprilTagStartPose,
        startToParkControl1,
        startToParkControl2,
        parkEndPose
    ).constant(Math.toRadians(90.0))

    // Dynamic paths to parkEndPose maintaining the current heading of the preceding stop - all using Bezier curves!
    fun pathToPark(fromPose: Pose, isBlueAlliance: Boolean = false): Path {
        val targetPark = if (isBlueAlliance) blueParkEndPose else parkEndPose
        val targetParkPose = poseFactory.of(targetPark.x(), targetPark.y(), Math.toDegrees(fromPose.heading()))
        return if (isBlueAlliance) {
            // Blue park is in Tile [Col 6, Row 2] (Y: 24 to 48) near the alliance wall
            if (fromPose.y() < 30.0) {
                // Bezier curve from start (81, 9) out to (131, 39)
                Paths.curve(fromPose, blueStartToParkControl1, blueStartToParkControl2, targetParkPose).constant(fromPose.heading())
            } else {
                // Bezier curve from opposite launch (89, 125) around right side of center hive down to (131, 39)
                Paths.curve(fromPose, blueOppositeToParkControl1, blueOppositeToParkControl2, targetParkPose).constant(fromPose.heading())
            }
        } else {
            // Red park is in Tile [Col 1, Row 5] (Y: 96 to 120)
            if (fromPose.y() < 30.0) {
                // From start (63, 9) up to (13, 105), curve around the left of the hive
                Paths.curve(fromPose, startToParkControl1, startToParkControl2, targetParkPose).constant(fromPose.heading())
            } else {
                // From opposite launch (55, 125) down to (13, 105) via Bezier curve
                Paths.curve(fromPose, redOppositeToParkControl1, redOppositeToParkControl2, targetParkPose).constant(fromPose.heading())
            }
        }
    }

    // =========================================================================
    // Blue Alliance Paths
    // Start & Intermediate are mirrored across X = 72 (X' = 144 - X)
    // Blue Observation / Park is 2nd tile from bottom: Row 2 (Y: 24" to 48"), Col 6 (X: 120" to 144")
    // Center at X = 131.0, Y = 39.0 gives 4.0" clearance to right wall (X=144) and 6.0" clearance to Row 1 (Y=24)
    // =========================================================================
    val blueAprilTagStartPose: Pose = poseFactory.of(144.0 - 63.0, 9.0, 90.0) // (81.0, 9.0, 90.0°)
    val blueAprilTagIntermediatePose: Pose = poseFactory.of(144.0 - 63.0, 125.0, 90.0) // (81.0, 125.0, 90.0°)
    val blueAprilTagFinalPose: Pose = poseFactory.of(144.0 - 55.0, 125.0, 270.0) // (89.0, 125.0, 270.0°)
    val blueParkEndPose: Pose = poseFactory.of(131.0, 39.0, 90.0) // 2nd tile from bottom, Col 6

    // Blue Aimed Poses (Mirrored: 180° - theta)
    // Blue Start Aim: 180° - 94.3° = 85.7°
    val blueStartAimPose: Pose = poseFactory.of(81.0, 9.0, 85.7)
    val blueStartAimPath: Path get() = Paths.line(blueAprilTagStartPose, blueStartAimPose).linear(Math.toRadians(90.0), Math.toRadians(85.7))
    val blueStartRealignPath: Path get() = Paths.line(blueStartAimPose, blueAprilTagStartPose).linear(Math.toRadians(85.7), Math.toRadians(90.0))

    // Blue Opposite Aim: 180° - 278.0° = -98.0° = 262.0°
    val blueOppositeAimPose: Pose = poseFactory.of(89.0, 125.0, 262.0)
    val blueOppositeAimPath: Path get() = Paths.line(blueAprilTagFinalPose, blueOppositeAimPose).linear(Math.toRadians(270.0), Math.toRadians(262.0))
    val blueOppositeRealignPath: Path get() = Paths.line(blueOppositeAimPose, blueAprilTagFinalPose).linear(Math.toRadians(262.0), Math.toRadians(270.0))

    val blueNorthControl1: Pose = poseFactory.of(81.0, 67.0, 90.0)
    val blueAprilTagIntermediatePath: Path get() = Paths.curve(blueAprilTagStartPose, blueNorthControl1, blueAprilTagIntermediatePose).constant(Math.toRadians(90.0))
    
    val blueOppositeCurveControl: Pose = poseFactory.of(85.0, 125.0, 180.0)
    val blueAprilTagFinalPath: Path get() = Paths.curve(blueAprilTagIntermediatePose, blueOppositeCurveControl, blueAprilTagFinalPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    // Blue control points: The metal bars on the right of the center structure run along X=96 (Y: 48 to 96).
    val blueStartToParkControl1: Pose = poseFactory.of(106.0, 18.0, 90.0)
    val blueStartToParkControl2: Pose = poseFactory.of(126.0, 24.0, 90.0)

    val blueOppositeToParkControl1: Pose = poseFactory.of(126.0, 75.0, 270.0)
    val blueOppositeToParkControl2: Pose = poseFactory.of(126.0, 45.0, 270.0)
}

