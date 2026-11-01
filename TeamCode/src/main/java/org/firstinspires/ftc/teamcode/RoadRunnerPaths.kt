package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d

/**
 * Defines standard paths for the autonomous routine using Road Runner 1.0.
 */
class RoadRunnerPaths(private val drive: TankDrive) {

    // 1. Translated Poses (X, Y, Heading in radians)
    val startPose = Pose2d(56.0, 8.0, Math.toRadians(90.0))
    val shootingPosition = Pose2d(56.0, 36.0, Math.toRadians(90.0))
    val reloadingStart = Pose2d(34.8008, 10.97, Math.toRadians(180.0))
    val loadGarden = Pose2d(0.8333, 14.1918, Math.toRadians(-5.4183))
    val oppositeShootingPosition = Pose2d(52.8239, 126.8941, Math.toRadians(270.0))

    // 2. Translated Paths

    fun shootingPositionPath(): Action {
        // Pedro equivalent: Paths.line(start, shootingposition).linear(start, shootingposition)
        return drive.actionBuilder(startPose)
            .lineToY(36.0) // Straight line along Y-axis, keeping 90 degree heading
            .build()
    }

    fun reloadingStartPath(): Action {
        // Pedro equivalent: Paths.line(shootingposition, reloadingstart).constant(reloadingstart)
        return drive.actionBuilder(shootingPosition)
            .splineTo(reloadingStart.position, reloadingStart.heading)
            .build()
    }

    fun loadGardenPath(): Action {
        // Pedro equivalent: Paths.line(reloadingstart, loadGarden).reverseTangent()
        return drive.actionBuilder(reloadingStart)
            .setReversed(true) // Road Runner equivalent of reverseTangent()
            .splineTo(loadGarden.position, loadGarden.heading)
            .build()
    }

    fun oppositeShootingPositionPath(): Action {
        // Pedro equivalent: Paths.line(loadGarden, oppositeshootingposition).constant(oppositeshootingposition)
        return drive.actionBuilder(loadGarden)
            .setReversed(false) // Ensure we are driving forward again
            .splineTo(oppositeShootingPosition.position, oppositeShootingPosition.heading)
            .build()
    }
}
