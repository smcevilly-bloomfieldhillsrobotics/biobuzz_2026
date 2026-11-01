package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.pedropathing.paths.Path

/**
 * Defines standard paths for the autonomous routine using Road Runner 1.0 and Pedro Pathing 3.0.
 * Serves as a unified path definition class for both pathing systems.
 */
class RoadRunnerPaths(private val drive: TankDrive) {

    /** Pedro Pathing 3.0 equivalent paths instance */
    val pedroPaths = PedroPaths()

    // 1. Translated Poses (X, Y, Heading in radians)
    val startPose = Pose2d(56.0, 8.0, Math.toRadians(90.0))
    val shootingPosition = Pose2d(56.0, 36.0, Math.toRadians(90.0))
    val reloadingStart = Pose2d(34.8008, 10.97, Math.toRadians(180.0))
    val loadGarden = Pose2d(0.8333, 14.1918, Math.toRadians(-5.4183))
    val oppositeShootingPosition = Pose2d(52.8239, 126.8941, Math.toRadians(270.0))

    // 2. Road Runner Actions

    fun shootingPositionPath(): Action {
        return drive.actionBuilder(startPose)
            .lineToY(36.0) // Straight line along Y-axis, keeping 90 degree heading
            .build()
    }

    fun reloadingStartPath(): Action {
        return drive.actionBuilder(shootingPosition)
            .splineTo(reloadingStart.position, reloadingStart.heading)
            .build()
    }

    fun loadGardenPath(): Action {
        return drive.actionBuilder(reloadingStart)
            .setReversed(true)
            .splineTo(loadGarden.position, loadGarden.heading)
            .build()
    }

    fun oppositeShootingPositionPath(): Action {
        return drive.actionBuilder(loadGarden)
            .setReversed(false)
            .splineTo(oppositeShootingPosition.position, oppositeShootingPosition.heading)
            .build()
    }

    // 3. Pedro Pathing 3.0 Paths (Direct Delegate)

    fun pedroShootingPositionPath(): Path = pedroPaths.shootingPositionPath()
    fun pedroReloadingStartPath(): Path = pedroPaths.reloadingStartPath()
    fun pedroLoadGardenPath(): Path = pedroPaths.loadGardenPath()
    fun pedroOppositeShootingPositionPath(): Path = pedroPaths.oppositeShootingPositionPath()
}

