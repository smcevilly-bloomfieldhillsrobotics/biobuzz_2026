package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * BioBuzz Mecanum TeleOp with Odometry tracking.
 * Features full holonomic movement (forward, strafe, turn) and optional field-centric drive.
 * Designed to integrate seamlessly with Pedro Pathing or Road Runner Pinpoint odometry.
 */
@TeleOp(name = "BioBuzz Mecanum Teleop (Pedro / Pinpoint)", group = "StarterBot")
class BioBuzzMecanumTeleop : OpMode() {

    private val robot = RobotHardware()
    private lateinit var mecanumDrive: MecanumDrivetrainSubsystem
    private lateinit var intakeLauncher: IntakeLauncherSubsystem
    private lateinit var odometry: OdometrySubsystem

    // Field-centric vs Robot-centric toggle
    private var fieldCentric = true
    private var lastAButton = false

    // Tuning buttons edge detection
    private var lastDpadUp = false
    private var lastDpadDown = false

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)
        robot.initLimelight(hardwareMap)
        robot.initPinpoint(hardwareMap)

        mecanumDrive = MecanumDrivetrainSubsystem(robot)
        intakeLauncher = IntakeLauncherSubsystem(robot)
        odometry = OdometrySubsystem(robot)

        odometry.resetPosition(0.0, 0.0, 0.0)

        telemetry.addData("Status", "Mecanum TeleOp Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Driver 1: Holonomic Mecanum Drive
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

        if (fieldCentric) {
            val headingRad = Math.toRadians(odometry.heading)
            mecanumDrive.driveFieldRelative(forward, strafe, rotate, headingRad)
        } else {
            mecanumDrive.drive(forward, strafe, rotate)
        }

        // Driver 2: Intake & Single Launcher
        // RT: Launcher Forward, LT: Launcher Reverse
        // RB: Intake Forward, LB: Intake Reverse
        intakeLauncher.update(
            gamepad2.right_trigger,
            gamepad2.left_trigger,
            gamepad2.right_bumper,
            gamepad2.left_bumper
        )

        // Driver 2: Tune Single Launcher Velocity with D-Pad
        if (gamepad2.dpad_up && !lastDpadUp) {
            intakeLauncher.adjustLauncher1Velocity(50.0)
        }
        if (gamepad2.dpad_down && !lastDpadDown) {
            intakeLauncher.adjustLauncher1Velocity(-50.0)
        }
        lastDpadUp = gamepad2.dpad_up
        lastDpadDown = gamepad2.dpad_down

        // Update Odometry and stream telemetry to FTC Dashboard
        val launcherVel = robot.launcher?.velocity ?: 0.0

        val packet = TelemetryPacket()
        packet.put("Drive Mode", if (fieldCentric) "Field-Centric" else "Robot-Centric")
        packet.put("Launcher Vel", launcherVel)
        packet.put("Launcher Target", IntakeLauncherSubsystem.TARGET_VELOCITY)
        packet.put("Intake Power", intakeLauncher.intakePower)
        packet.put("FL Power", mecanumDrive.frontLeftPower)
        packet.put("FR Power", mecanumDrive.frontRightPower)
        packet.put("BL Power", mecanumDrive.backLeftPower)
        packet.put("BR Power", mecanumDrive.backRightPower)

        odometry.update(packet)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        // Telemetry
        telemetry.addData("--- BioBuzz Mecanum Teleop ---", "")
        telemetry.addData("Drive Mode", if (fieldCentric) "Field-Centric (Press A to toggle)" else "Robot-Centric (Press A to toggle)")
        telemetry.addData("Motors (FL, FR, BL, BR)", "%.2f, %.2f, %.2f, %.2f",
            mecanumDrive.frontLeftPower, mecanumDrive.frontRightPower,
            mecanumDrive.backLeftPower, mecanumDrive.backRightPower
        )
        telemetry.addData("Launcher Vel", "%.1f ticks/sec (Target: %.0f)", launcherVel, IntakeLauncherSubsystem.TARGET_VELOCITY)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)
        telemetry.addData("--- Odometry (Pinpoint / Pedro) ---", "")
        telemetry.addData("X Position (in)", "%.2f", odometry.posX)
        telemetry.addData("Y Position (in)", "%.2f", odometry.posY)
        telemetry.addData("Heading (deg)", "%.2f", odometry.heading)
        telemetry.update()
    }

    override fun stop() {
        mecanumDrive.stop()
        intakeLauncher.stop()
    }
}
