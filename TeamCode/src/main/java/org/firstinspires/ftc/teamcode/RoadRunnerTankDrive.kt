package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/**
 * Road Runner 1.0 Tank Drive implementation tailored for the BioBuzz StarterBot
 * with native goBILDA Pinpoint Odometry localization.
 */
@Config
class RoadRunnerTankDrive(val robot: RobotHardware) {

    companion object {
        @JvmField var TRACK_WIDTH_INCHES = 14.5
    }

    var pose: Pose2d = Pose2d(0.0, 0.0, 0.0)

    fun updatePose(): Pose2d {
        val pinpoint = robot.pinpoint ?: return pose
        pinpoint.update()
        val pos = pinpoint.position
        val headingRad = pos.getHeading(AngleUnit.RADIANS)
        pose = Pose2d(
            pos.getX(DistanceUnit.INCH),
            pos.getY(DistanceUnit.INCH),
            headingRad
        )
        return pose
    }

    fun setDrivePowers(leftPower: Double, rightPower: Double) {
        robot.leftDrive?.power = leftPower
        robot.rightDrive?.power = rightPower
    }

    fun stop() {
        setDrivePowers(0.0, 0.0)
    }
}
