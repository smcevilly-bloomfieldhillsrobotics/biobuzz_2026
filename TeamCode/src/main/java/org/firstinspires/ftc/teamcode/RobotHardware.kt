/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode

import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.drivetrains.MecanumConfig
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/**
 * RobotHardware written in idiomatic Kotlin, featuring multi-name fallbacks 
 * for
 * motors, servos, Limelight, and GoBilda Pinpoint Odometry Computer.
 */
class RobotHardware {

    companion object {
        val FRONT_LEFT_DRIVE_NAMES = arrayOf("front_left_drive", "leftFront", "frontLeft", "motor0")
        val FRONT_RIGHT_DRIVE_NAMES = arrayOf("front_right_drive", "rightFront", "frontRight", "motor1")
        val BACK_LEFT_DRIVE_NAMES = arrayOf("back_left_drive", "leftRear", "backLeft", "motor2")
        val BACK_RIGHT_DRIVE_NAMES = arrayOf("back_right_drive", "rightRear", "backRight", "motor3")
        val INTAKE_NAMES = arrayOf("intake", "intake_motor", "motor2")
        // Nectar Launcher = Right Launcher (Motor 3 / Right)
        val NECTAR_LAUNCHER_NAMES = arrayOf("nectar_launcher", "nectarLauncher", "launcher", "launcher_motor", "motor3", "motor_3", "right_launcher", "rightLauncher")
        // Pollen Launcher = Left Launcher (Motor 4 / Left)
        val POLLEN_LAUNCHER_NAMES = arrayOf("pollen_launcher", "pollenLauncher", "launcher2", "launcher_motor_2", "motor4", "launcher_2", "left_launcher", "leftLauncher")
        val CAMERA_MOTOR_NAMES = arrayOf("camera_motor", "motor3", "motor_3", "cameraMotor", "pan_motor")
        val POLLEN_LAUNCHER_SERVO_NAMES = arrayOf("pollen_launcher_servo", "pollenLauncherServo", "left_launcher_servo", "leftLauncherServo", "left_intake_servo", "leftIntakeServo", "left_servo", "leftServo", "servo1", "servo3")
        val NECTAR_LAUNCHER_SERVO_NAMES = arrayOf("nectar_launcher_servo", "nectarLauncherServo", "right_launcher_servo", "rightLauncherServo", "right_intake_servo", "rightIntakeServo", "right_servo", "rightServo", "servo2", "servo4")
        val LIMELIGHT_NAMES = arrayOf("limelight", "limelight3a", "camera")
        val PINPOINT_NAMES = arrayOf("pinpoint", "odometry", "pinpoint_odom")
    }

    var frontLeftDrive: DcMotor? = null
    var frontRightDrive: DcMotor? = null
    var backLeftDrive: DcMotor? = null
    var backRightDrive: DcMotor? = null
    var intake: DcMotor? = null
    var nectarLauncher: DcMotorEx? = null
    var pollenLauncher: DcMotorEx? = null
    var cameraMotor: DcMotor? = null
    var nectarLauncherServo: CRServo? = null
    var pollenLauncherServo: CRServo? = null

    // Backward-compatible aliases
    val launcher: DcMotorEx? get() = nectarLauncher
    val launcher2: DcMotorEx? get() = pollenLauncher
    val rightLauncherServo: CRServo? get() = nectarLauncherServo
    val leftLauncherServo: CRServo? get() = pollenLauncherServo
    val rightServo: CRServo? get() = nectarLauncherServo
    val leftServo: CRServo? get() = pollenLauncherServo
    var limelight: Limelight3A? = null
    var pinpoint: GoBildaPinpointDriver? = null

    fun init(hardwareMap: HardwareMap) {
        initAll(hardwareMap)
    }

    fun initAll(hardwareMap: HardwareMap) {
        initMecanumDrivetrain(hardwareMap)
        initIntakeAndLauncher(hardwareMap)
        initLimelight(hardwareMap)
        initPinpoint(hardwareMap)
    }

    fun initMecanumDrivetrain(hardwareMap: HardwareMap) {
        frontLeftDrive = tryGetMotor(hardwareMap, *FRONT_LEFT_DRIVE_NAMES)?.apply {
            direction = DcMotorSimple.Direction.REVERSE
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        frontRightDrive = tryGetMotor(hardwareMap, *FRONT_RIGHT_DRIVE_NAMES)?.apply {
            direction = DcMotorSimple.Direction.REVERSE
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        backLeftDrive = tryGetMotor(hardwareMap, *BACK_LEFT_DRIVE_NAMES)?.apply {
            direction = DcMotorSimple.Direction.REVERSE
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        backRightDrive = tryGetMotor(hardwareMap, *BACK_RIGHT_DRIVE_NAMES)?.apply {
            direction = DcMotorSimple.Direction.FORWARD
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    /**
     * Centralized Pedro Pathing Mecanum Drivetrain factory.
     * Reuses motor instances, configurations, and directions defined in RobotHardware.
     */
    fun createPedroMecanum(hardwareMap: HardwareMap): Mecanum {
        if (frontLeftDrive == null || frontRightDrive == null || backLeftDrive == null || backRightDrive == null) {
            initMecanumDrivetrain(hardwareMap)
        }

        val flName = frontLeftDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: FRONT_LEFT_DRIVE_NAMES[0]
        val frName = frontRightDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: FRONT_RIGHT_DRIVE_NAMES[0]
        val blName = backLeftDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: BACK_LEFT_DRIVE_NAMES[0]
        val brName = backRightDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: BACK_RIGHT_DRIVE_NAMES[0]

        val config = MecanumConfig { c ->
            c.frontLeftName.set(flName)
            c.frontRightName.set(frName)
            c.backLeftName.set(blName)
            c.backRightName.set(brName)

            c.frontLeftDirection.set(frontLeftDrive?.direction ?: DcMotorSimple.Direction.REVERSE)
            c.frontRightDirection.set(frontRightDrive?.direction ?: DcMotorSimple.Direction.REVERSE)
            c.backLeftDirection.set(backLeftDrive?.direction ?: DcMotorSimple.Direction.REVERSE)
            c.backRightDirection.set(backRightDrive?.direction ?: DcMotorSimple.Direction.FORWARD)
        }

        return Mecanum(hardwareMap, config)
    }

    fun initIntakeAndLauncher(hardwareMap: HardwareMap) {
        intake = tryGetMotor(hardwareMap, *INTAKE_NAMES)?.apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        nectarLauncher = tryGetMotorEx(hardwareMap, *NECTAR_LAUNCHER_NAMES)?.apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        pollenLauncher = tryGetMotorEx(hardwareMap, *POLLEN_LAUNCHER_NAMES)?.apply {
            direction = DcMotorSimple.Direction.REVERSE
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        nectarLauncherServo = tryGetCRServo(hardwareMap, *NECTAR_LAUNCHER_SERVO_NAMES)?.apply {
            direction = DcMotorSimple.Direction.REVERSE
            power = 0.0
        }

        pollenLauncherServo = tryGetCRServo(hardwareMap, *POLLEN_LAUNCHER_SERVO_NAMES)?.apply {
            direction = DcMotorSimple.Direction.FORWARD
            power = 0.0
        }
    }

    fun initLimelight(hardwareMap: HardwareMap) {
        cameraMotor = tryGetMotor(hardwareMap, *CAMERA_MOTOR_NAMES)?.apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        for (name in LIMELIGHT_NAMES) {
            try {
                limelight = hardwareMap[Limelight3A::class.java, name]
                if (limelight != null) {
                    limelight?.setPollRateHz(100)
                    limelight?.start()
                    limelight?.pipelineSwitch(0)
                    break
                }
            } catch (_: Exception) {
            }
        }
    }

    fun initPinpoint(hardwareMap: HardwareMap) {
        for (name in PINPOINT_NAMES) {
            try {
                pinpoint = hardwareMap[GoBildaPinpointDriver::class.java, name]?.apply {
                    setOffsets(OdometrySubsystem.X_POD_OFFSET_MM, OdometrySubsystem.Y_POD_OFFSET_MM, DistanceUnit.MM)
                    setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.REVERSED,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    )
                    recalibrateIMU()
                }
                if (pinpoint != null) break
            } catch (_: Exception) {
            }
        }
    }

    private fun tryGetMotor(hardwareMap: HardwareMap, vararg names: String): DcMotor? {
        for (name in names) {
            try {
                val motor = hardwareMap[DcMotor::class.java, name]
                if (motor != null) return motor
            } catch (_: Exception) {
            }
        }
        return null
    }

    private fun tryGetMotorEx(hardwareMap: HardwareMap, vararg names: String): DcMotorEx? {
        for (name in names) {
            try {
                val motor = hardwareMap[DcMotorEx::class.java, name]
                if (motor != null) return motor
            } catch (_: Exception) {
            }
        }
        return null
    }

    private fun tryGetCRServo(hardwareMap: HardwareMap, vararg names: String): CRServo? {
        for (name in names) {
            try {
                val servo = hardwareMap[CRServo::class.java, name]
                if (servo != null) return servo
            } catch (_: Exception) {
            }
        }
        return null
    }
}
