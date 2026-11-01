package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.drivetrains.MecanumConfig
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.cos
import kotlin.math.sin

/**
 * BioBuzz Mecanum TeleOp powered directly by Pedro Pathing's Mecanum Drivetrain & Pinpoint Odometry.
 * Features full holonomic movement (forward, strafe, turn) and optional field-centric drive.
 */
@TeleOp(name = "BioBuzz Mecanum Teleop (Pedro)", group = "StarterBot")
class BioBuzzMecanumTeleop : OpMode() {

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var intakeLauncher: IntakeLauncherSubsystem
    private lateinit var odometry: OdometrySubsystem

    // Field-centric vs Robot-centric toggle
    private var fieldCentric = true
    private var lastAButton = false

    // Tuning buttons edge detection
    private var lastDpadUp = false
    private var lastDpadDown = false
    private var lastDpadRight = false
    private var lastDpadLeft = false

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)
        robot.initLimelight(hardwareMap)
        robot.initPinpoint(hardwareMap)

        // Find configured hardware names for the 4 mecanum drive motors
        val flName = robot.frontLeftDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: "front_left_drive"
        val frName = robot.frontRightDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: "front_right_drive"
        val blName = robot.backLeftDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: "back_left_drive"
        val brName = robot.backRightDrive?.let { hardwareMap.getNamesOf(it).firstOrNull() } ?: "back_right_drive"

        // Initialize Pedro Pathing's Mecanum Drivetrain
        val config = MecanumConfig { c ->
            c.frontLeftName.set(flName)
            c.frontRightName.set(frName)
            c.backLeftName.set(blName)
            c.backRightName.set(brName)

            c.frontLeftDirection.set(robot.frontLeftDrive?.direction ?: DcMotorSimple.Direction.REVERSE)
            c.frontRightDirection.set(robot.frontRightDrive?.direction ?: DcMotorSimple.Direction.FORWARD)
            c.backLeftDirection.set(robot.backLeftDrive?.direction ?: DcMotorSimple.Direction.REVERSE)
            c.backRightDirection.set(robot.backRightDrive?.direction ?: DcMotorSimple.Direction.REVERSE)
        }
        pedroMecanum = Mecanum(hardwareMap, config)

        intakeLauncher = IntakeLauncherSubsystem(robot)
        odometry = OdometrySubsystem(robot)

        odometry.resetPosition(0.0, 0.0, 0.0)

        telemetry.addData("Status", "Pedro Mecanum TeleOp Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Driver 1: Holonomic Mecanum Drive (Pedro Pathing DrivePowers)
        val forward = -gamepad1.left_stick_y.toDouble()
        val strafe = gamepad1.left_stick_x.toDouble()
        val rotate = gamepad1.right_stick_x.toDouble()

        // Toggle Field-Centric drive mode with Gamepad 1 A
        if (gamepad1.a && !lastAButton) {
            fieldCentric = !fieldCentric
        }
        lastAButton = gamepad1.a

        // Reset Heading / Odometry with BACK button
        if (gamepad1.back) {
            odometry.resetPosition(0.0, 0.0, 0.0)
        }

        // Apply Pedro Pathing DrivePowers (Field-Relative or Robot-Centric)
        if (fieldCentric) {
            val headingRad = Math.toRadians(odometry.heading)
            val rotX = (strafe * cos(-headingRad)) - (forward * sin(-headingRad))
            val rotY = (strafe * sin(-headingRad)) + (forward * cos(-headingRad))
            pedroMecanum.drive(DrivePowers(rotY, rotX, rotate), true)
        } else {
            pedroMecanum.drive(DrivePowers(forward, strafe, rotate), true)
        }

        // Driver 2: Dual Launcher & Intake controls
        // RT: Spin Right Launcher to target speed (pulses right servo for 0.2s after hitting speed)
        // LT: Spin Left Launcher to target speed (pulses left servo for 0.2s after hitting speed)
        // RB: Intake Forward
        // LB: Reverse Intake & Servos (unjam)
        intakeLauncher.update(
            gamepad2.right_trigger,
            gamepad2.left_trigger,
            gamepad2.right_bumper,
            gamepad2.left_bumper
        )

        // Driver 2: Tune Launcher Velocities with D-Pad
        // D-Pad Up/Down: Right Launcher velocity (adjustLauncher1Velocity)
        // D-Pad Left/Right: Left Launcher velocity (adjustLauncher2Velocity)
        if (gamepad2.dpad_up && !lastDpadUp) {
            intakeLauncher.adjustLauncher1Velocity(50.0)
        }
        if (gamepad2.dpad_down && !lastDpadDown) {
            intakeLauncher.adjustLauncher1Velocity(-50.0)
        }
        lastDpadUp = gamepad2.dpad_up
        lastDpadDown = gamepad2.dpad_down

        if (gamepad2.dpad_right && !lastDpadRight) {
            intakeLauncher.adjustLauncher2Velocity(50.0)
        }
        if (gamepad2.dpad_left && !lastDpadLeft) {
            intakeLauncher.adjustLauncher2Velocity(-50.0)
        }
        lastDpadRight = gamepad2.dpad_right
        lastDpadLeft = gamepad2.dpad_left

        // Update Odometry and stream telemetry to FTC Dashboard
        val rightLauncherVel = robot.launcher?.velocity ?: 0.0
        val leftLauncherVel = robot.launcher2?.velocity ?: 0.0

        val wheelPowers = pedroMecanum.wheelPowers
        val flPower = if (wheelPowers.isNotEmpty()) wheelPowers[0] else 0.0
        val frPower = if (wheelPowers.size > 1) wheelPowers[1] else 0.0
        val blPower = if (wheelPowers.size > 2) wheelPowers[2] else 0.0
        val brPower = if (wheelPowers.size > 3) wheelPowers[3] else 0.0

        val packet = TelemetryPacket()
        packet.put("Drive Mode", if (fieldCentric) "Field-Centric" else "Robot-Centric")
        packet.put("Right Launcher Vel", rightLauncherVel)
        packet.put("Right Launcher Target", IntakeLauncherSubsystem.TARGET_VELOCITY)
        packet.put("Left Launcher Vel", leftLauncherVel)
        packet.put("Left Launcher Target", IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY)
        packet.put("Intake Power", intakeLauncher.intakePower)
        packet.put("FL Power", flPower)
        packet.put("FR Power", frPower)
        packet.put("BL Power", blPower)
        packet.put("BR Power", brPower)

        odometry.update(packet)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        // Telemetry
        telemetry.addData("--- BioBuzz Mecanum Teleop (Pedro) ---", "")
        telemetry.addData("Drive Mode", if (fieldCentric) "Field-Centric (Press A to toggle)" else "Robot-Centric (Press A to toggle)")
        telemetry.addData("Motors (FL, FR, BL, BR)", "%.2f, %.2f, %.2f, %.2f",
            flPower, frPower, blPower, brPower
        )
        telemetry.addData("Controls", "RT: Right Launcher, LT: Left Launcher, RB: Intake, LB: Reverse/Unjam")
        telemetry.addData("Right Launcher Vel", "%.1f ticks/sec (Target: %.0f)", rightLauncherVel, IntakeLauncherSubsystem.TARGET_VELOCITY)
        telemetry.addData("Left Launcher Vel", "%.1f ticks/sec (Target: %.0f)", leftLauncherVel, IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)
        telemetry.addData("--- Odometry (Pinpoint / Pedro) ---", "")
        telemetry.addData("X Position (in)", "%.2f", odometry.posX)
        telemetry.addData("Y Position (in)", "%.2f", odometry.posY)
        telemetry.addData("Heading (deg)", "%.2f", odometry.heading)
        telemetry.update()
    }

    override fun stop() {
        pedroMecanum.stop()
        intakeLauncher.stop()
    }
}
