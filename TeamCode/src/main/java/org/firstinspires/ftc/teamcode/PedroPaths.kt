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

    // Poses defined with PoseFactory (X, Y, Heading in degrees)
    val start: Pose = poseFactory.of(56.0, 8.0, 90.0)
    val startPose: Pose get() = start

    val startPath: Pose = poseFactory.of(56.0, 36.0, 0.0)
    val shootingPosition: Pose get() = startPath

    val reloadingStartPathStart: Pose = poseFactory.of(56.0, 36.0, 90.0)
    val reloadingStartPathControl1: Pose = poseFactory.of(56.0, 20.0, 0.0)
    val reloadingStartPath: Pose = poseFactory.of(34.8008, 10.97, 180.0)
    val reloadingStart: Pose get() = reloadingStartPath

    val loadGardenPathStart: Pose = poseFactory.of(34.8008, 10.97, 180.0)
    val loadGardenPathControl1: Pose = poseFactory.of(17.8, 12.58, 0.0)
    val loadGardenPath: Pose = poseFactory.of(0.8333, 14.1918, -5.4183)
    val loadGarden: Pose get() = loadGardenPath

    val oppositeShootingPositionPathStart: Pose = poseFactory.of(0.8333, 14.1918, -5.4183)
    // Route safely along the left perimeter wall (X=10.0, well away from Red Hive at X=46.0) up to Y=85.0 before cutting in
    val oppositeShootingPositionPathControl1: Pose = poseFactory.of(10.0, 85.0, 0.0)
    val oppositeShootingPositionPath: Pose = poseFactory.of(52.8239, 126.8941, 270.0)
    val oppositeShootingPosition: Pose get() = oppositeShootingPositionPath

    // Trajectory functions configured for constant 90° heading throughout

    fun startPath(): Path =
        Paths.line(start, startPath).constant(Math.toRadians(90.0))

    fun shootingPositionPath(): Path = startPath()

    fun reloadingStartPath(): Path =
        Paths.curve(reloadingStartPathStart, reloadingStartPathControl1, reloadingStartPath)
            .constant(Math.toRadians(90.0))

    fun loadGardenPath(): Path =
        Paths.curve(loadGardenPathStart, loadGardenPathControl1, loadGardenPath)
            .constant(Math.toRadians(90.0))

    fun oppositeShootingPositionPath(): Path =
        Paths.curve(
            oppositeShootingPositionPathStart,
            oppositeShootingPositionPathControl1,
            oppositeShootingPositionPath
        ).linear(Math.toRadians(90.0), Math.toRadians(270.0))
}

