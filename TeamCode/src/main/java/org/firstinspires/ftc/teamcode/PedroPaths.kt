package org.firstinspires.ftc.teamcode

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path

/**
 * Defines standard paths for the autonomous routine using Pedro Pathing 3.0 with PoseFactory.degrees().
 * Poses are specified in degrees directly without manual radian conversion:
 *
 * 1. startPose:                 (56.0, 8.0, 90.0°)
 * 2. shootingPosition:          (56.0, 36.0, 90.0°)
 * 3. reloadingStart:            (34.8008, 10.97, 180.0°)
 * 4. loadGarden:                (0.8333, 14.1918, -5.4183°)
 * 5. oppositeShootingPosition:  (52.8239, 126.8941, 270.0°)
 */
class PedroPaths {

    private val poseFactory = PoseFactory.degrees()

    // 1. Defined Poses using PoseFactory (X, Y, Heading in degrees)
    val startPose: Pose = poseFactory.of(56.0, 8.0, 90.0)
    val shootingPosition: Pose = poseFactory.of(56.0, 36.0, 90.0)
    val reloadingStart: Pose = poseFactory.of(34.8008, 10.97, 180.0)
    val loadGarden: Pose = poseFactory.of(0.8333, 14.1918, -5.4183)
    val oppositeShootingPosition: Pose = poseFactory.of(52.8239, 126.8941, 270.0)

    // 2. Trajectories matching the autonomous routine

    fun shootingPositionPath(): Path =
        Paths.line(startPose, shootingPosition).constant(shootingPosition)

    fun reloadingStartPath(): Path =
        Paths.curve(startPose, shootingPosition, reloadingStart).linear(shootingPosition, reloadingStart)

    fun loadGardenPath(): Path =
        Paths.curve(reloadingStart, loadGarden).reverseTangent()

    fun oppositeShootingPositionPath(): Path =
        Paths.curve(loadGarden, oppositeShootingPosition).linear(loadGarden, oppositeShootingPosition)
}

