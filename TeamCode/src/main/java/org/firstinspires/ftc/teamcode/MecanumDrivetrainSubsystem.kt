package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.math.max

/**
 * Legacy MecanumDrivetrainSubsystem.
 * Deprecated in favor of Pedro Pathing's native Mecanum drivetrain:
 *   robot.createPedroMecanum(hardwareMap)
 */
@Deprecated(
    message = "Use Pedro Pathing's native Mecanum drivetrain via robot.createPedroMecanum(hardwareMap) instead.",
    replaceWith = ReplaceWith("robot.createPedroMecanum(hardwareMap)")
)
class MecanumDrivetrainSubsystem(private val robot: RobotHardware) {

    var frontLeftPower: Double = 0.0
        private set
    var frontRightPower: Double = 0.0
        private set
    var backLeftPower: Double = 0.0
        private set
    var backRightPower: Double = 0.0
        private set

    fun drive(forward: Double, strafe: Double, rotate: Double) {
        val fl = forward - strafe + rotate
        val bl = forward + strafe + rotate
        val fr = forward + strafe - rotate
        val br = forward - strafe - rotate

        val maxPower = max(1.0, max(max(abs(fl), abs(bl)), max(abs(fr), abs(br))))

        frontLeftPower = fl / maxPower
        backLeftPower = bl / maxPower
        frontRightPower = fr / maxPower
        backRightPower = br / maxPower

        robot.frontLeftDrive?.power = frontLeftPower
        robot.backLeftDrive?.power = backLeftPower
        robot.frontRightDrive?.power = frontRightPower
        robot.backRightDrive?.power = backRightPower
    }

    fun driveFieldRelative(forward: Double, strafe: Double, rotate: Double, headingRad: Double) {
        val rotX = strafe * Math.cos(-headingRad) - forward * Math.sin(-headingRad)
        val rotY = strafe * Math.sin(-headingRad) + forward * Math.cos(-headingRad)
        drive(rotY, rotX, rotate)
    }

    fun stop() {
        frontLeftPower = 0.0
        frontRightPower = 0.0
        backLeftPower = 0.0
        backRightPower = 0.0
        robot.frontLeftDrive?.power = 0.0
        robot.frontRightDrive?.power = 0.0
        robot.backLeftDrive?.power = 0.0
        robot.backRightDrive?.power = 0.0
    }
}
