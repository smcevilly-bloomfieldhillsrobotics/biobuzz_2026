package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.math.max

/**
 * MecanumDrivetrainSubsystem encapsulates holonomic Mecanum wheel kinematics.
 * Supports robot-centric and field-centric drive.
 */
class MecanumDrivetrainSubsystem(private val robot: RobotHardware) {

    var frontLeftPower: Double = 0.0
        private set
    var frontRightPower: Double = 0.0
        private set
    var backLeftPower: Double = 0.0
        private set
    var backRightPower: Double = 0.0
        private set

    /**
     * Drives the robot holonomically using standard mecanum kinematics.
     * @param forward Forward translation (-1.0 to 1.0)
     * @param strafe Lateral translation (-1.0 to 1.0, right is positive)
     * @param rotate Rotational velocity (-1.0 to 1.0, clockwise is positive)
     */
    fun drive(forward: Double, strafe: Double, rotate: Double) {
        val fl = forward + strafe + rotate
        val bl = forward - strafe + rotate
        val fr = forward - strafe - rotate
        val br = forward + strafe - rotate

        // Normalize powers so no motor exceeds 1.0
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

    /**
     * Drives field-relative given current robot heading in radians.
     */
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
