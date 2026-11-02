package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

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

        pedroMecanum = robot.createPedroMecanum(hardwareMap)

        intakeLauncher = IntakeLauncherSubsystem(robot)
        odometry = OdometrySubsystem(robot)

        odometry.resetPosition(0.0, 0.0, 0.0)

        telemetry.addData("Status", "Pedro Mecanum TeleOp Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Driver 1: Holonomic Mecanum Drive (Pedro Pathing DrivePowers)
        val forward = -gamepad1.left_stick_y.toDouble()
        val strafe = -gamepad1.left_stick_x.toDouble() // Negated so gamepad right = right strafe in Pedro Pathing
        val rotate = -gamepad1.right_stick_x.toDouble() // Negated so gamepad right = right rotation (CW) in Pedro Pathing

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

        // Limelight AprilTag Detection
        val aprilTag = getAprilTagTarget()
        if (aprilTag != null && aprilTag.isValid) {
            packet.put("AprilTag ID", aprilTag.id)
            packet.put("AprilTag Distance (in)", aprilTag.distanceInches)
            packet.put("AprilTag Offset (°)", aprilTag.txDegrees)
        }

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

        // Limelight AprilTag Telemetry
        if (aprilTag != null && aprilTag.isValid) {
            val tagStr = if (aprilTag.id >= 0) "#${aprilTag.id}" else "Lock"
            telemetry.addData("Limelight AprilTag", "%s | Distance: %.1f in | Offset: %.1f°",
                tagStr, aprilTag.distanceInches, aprilTag.txDegrees)
        } else {
            telemetry.addData("Limelight AprilTag", "No AprilTag Target Detected")
        }

        telemetry.addData("--- Odometry (Pinpoint / Pedro) ---", "")
        telemetry.addData("X Position (in)", "%.2f", odometry.posX)
        telemetry.addData("Y Position (in)", "%.2f", odometry.posY)
        telemetry.addData("Heading (deg)", "%.2f", odometry.heading)
        telemetry.update()
    }

    data class AprilTagTarget(
        val id: Int,
        val txDegrees: Double,
        val distanceInches: Double,
        val isValid: Boolean
    )

    private fun getAprilTagTarget(): AprilTagTarget? {
        val limelight = robot.limelight ?: return null
        val status = limelight.status ?: return null
        if (status.fps <= 0.0) return null

        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null

        val fiducials = result.fiducialResults
        if (fiducials.isNullOrEmpty()) {
            if (result.tx != 0.0 || result.ty != 0.0) {
                val botpose = result.botpose
                val distInches = if (botpose != null) hypot(botpose.position.x, botpose.position.y) * 39.3701 else 0.0
                return AprilTagTarget(id = -1, txDegrees = result.tx, distanceInches = distInches, isValid = true)
            }
            return null
        }

        var bestTag: AprilTagTarget? = null
        var minDistance = Double.MAX_VALUE

        for (fr in fiducials) {
            val pose = fr.targetPoseCameraSpace
            val distInches = if (pose != null) sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z) * 39.3701 else 0.0

            if (distInches > 0.0 && distInches < minDistance) {
                minDistance = distInches
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            } else if (bestTag == null) {
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            }
        }

        return bestTag
    }

    override fun stop() {
        pedroMecanum.stop()
        intakeLauncher.stop()
    }
}
