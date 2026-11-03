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
    private lateinit var controls: DriverControls

    // Field-centric vs Robot-centric toggle
    private var fieldCentric = true

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)
        robot.initLimelight(hardwareMap)
        robot.initPinpoint(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)

        intakeLauncher = IntakeLauncherSubsystem(robot)
        odometry = OdometrySubsystem(robot)
        controls = DriverControls(GamepadEx(gamepad1), GamepadEx(gamepad2))

        odometry.resetPosition(0.0, 0.0, 0.0)

        telemetry.addData("Status", "Pedro Mecanum TeleOp Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Update centralized controls
        controls.update()

        // Driver 1: Toggle Field-Centric drive mode with Gamepad 1 A
        if (controls.toggleFieldCentric) {
            fieldCentric = !fieldCentric
        }

        // Driver 1: Reset Heading / Odometry with BACK button
        if (controls.resetOdometry) {
            odometry.resetPosition(0.0, 0.0, 0.0)
        }

        // Driver 1: Holonomic Drive Powers (Field-Relative or Robot-Centric)
        val powers = controls.drivePowers
        val fwdCmd = powers.forward()
        val strCmd = powers.strafe()
        val rotCmd = powers.turn()

        if (fieldCentric) {
            val headingRad = Math.toRadians(odometry.heading)
            val cosH = cos(headingRad)
            val sinH = sin(headingRad)

            // Convert Field-Frame vector (fwdCmd, strCmd) to Robot-Frame vector
            val robotFwd = fwdCmd * cosH + strCmd * sinH
            val robotStr = -fwdCmd * sinH + strCmd * cosH

            pedroMecanum.drive(DrivePowers(robotFwd, robotStr, rotCmd), true)
        } else {
            pedroMecanum.drive(powers, true)
        }

        // Driver 2: Dual Launcher & Intake controls
        // Button A: Spin up Nectar & Pollen launchers to target velocity
        // RT: Feed Nectar Launcher (Right) servo (only when Nectar launcher is at target speed)
        // LT: Feed Pollen Launcher (Left) servo (only when Pollen launcher is at target speed)
        // RB: Intake Forward
        // LB: Reverse Intake & Servos (unjam)
        intakeLauncher.update(
            controls.spinUpLaunchers,
            controls.nectarLauncherTrigger,
            controls.pollenLauncherTrigger,
            controls.intakeForward,
            controls.intakeReverse
        )

        // Driver 2: Tune Launcher Velocities with D-Pad
        if (controls.nectarVelocityUp) {
            intakeLauncher.adjustNectarLauncherVelocity(50.0)
        }
        if (controls.nectarVelocityDown) {
            intakeLauncher.adjustNectarLauncherVelocity(-50.0)
        }

        if (controls.pollenVelocityUp) {
            intakeLauncher.adjustPollenLauncherVelocity(50.0)
        }
        if (controls.pollenVelocityDown) {
            intakeLauncher.adjustPollenLauncherVelocity(-50.0)
        }

        // Update Odometry and stream telemetry to FTC Dashboard
        val nectarLauncherVel = robot.nectarLauncher?.velocity ?: 0.0
        val pollenLauncherVel = robot.pollenLauncher?.velocity ?: 0.0

        val wheelPowers = pedroMecanum.wheelPowers
        val flPower = if (wheelPowers.isNotEmpty()) wheelPowers[0] else 0.0
        val frPower = if (wheelPowers.size > 1) wheelPowers[1] else 0.0
        val blPower = if (wheelPowers.size > 2) wheelPowers[2] else 0.0
        val brPower = if (wheelPowers.size > 3) wheelPowers[3] else 0.0

        val packet = TelemetryPacket()
        packet.put("Drive Mode", if (fieldCentric) "Field-Centric" else "Robot-Centric")
        packet.put("Nectar Launcher Vel", nectarLauncherVel)
        packet.put("Nectar Launcher Target", IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        packet.put("Pollen Launcher Vel", pollenLauncherVel)
        packet.put("Pollen Launcher Target", IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
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
        telemetry.addData("Controls", "A: Spin Up, RT: Nectar Launcher, LT: Pollen Launcher, RB: Intake, LB: Reverse/Unjam")
        telemetry.addData("Nectar Launcher Vel", "%.1f ticks/sec (Target: %.0f)", nectarLauncherVel, IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        telemetry.addData("Pollen Launcher Vel", "%.1f ticks/sec (Target: %.0f)", pollenLauncherVel, IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
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
