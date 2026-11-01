package org.firstinspires.ftc.teamcode.pedrotuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.OdometrySubsystem
import org.firstinspires.ftc.teamcode.RobotHardware
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Pedro Pathing Tuning Step 3: Lateral (Strafe) Kinematics Tuning.
 *
 * Strafes left a specified distance (default: 48.0 inches) at STRAFE_POWER.
 * Measures max lateral velocity (in/sec) and strafe efficiency.
 */
@Config
@Autonomous(name = "3. Pedro Lateral Tuner", group = "Pedro Tuning")
class PedroLateralTuner : LinearOpMode() {

    companion object {
        @JvmField var TARGET_DISTANCE_INCHES = 48.0
        @JvmField var STRAFE_POWER = 0.50
    }

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var odometry: OdometrySubsystem

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initPinpoint(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)
        odometry = OdometrySubsystem(robot)

        odometry.resetPosition(0.0, 0.0, 0.0)

        telemetry.addData("Status", "Ready. Place robot with 48+ inches clearance to the LEFT.")
        telemetry.addData("Target Distance", "%.1f inches", TARGET_DISTANCE_INCHES)
        telemetry.update()

        waitForStart()

        if (!opModeIsActive()) return

        val startX = odometry.posX
        val startY = odometry.posY
        val startTime = System.currentTimeMillis()

        var maxVel = 0.0
        var lastY = startY
        var lastTime = startTime

        while (opModeIsActive()) {
            odometry.update(null)

            val currentX = odometry.posX
            val currentY = odometry.posY
            val distTraveled = hypot(currentX - startX, currentY - startY)

            val now = System.currentTimeMillis()
            val dt = (now - lastTime) / 1000.0
            if (dt > 0.005) {
                val instVel = abs(currentY - lastY) / dt
                if (instVel > maxVel) maxVel = instVel
                lastY = currentY
                lastTime = now
            }

            val packet = TelemetryPacket()
            packet.put("Lateral Distance Traveled", distTraveled)
            packet.put("Target Distance", TARGET_DISTANCE_INCHES)
            packet.put("Max Strafe Velocity (in/s)", maxVel)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            if (distTraveled >= TARGET_DISTANCE_INCHES) {
                break
            }

            // Strafe LEFT at STRAFE_POWER
            pedroMecanum.drive(DrivePowers(0.0, STRAFE_POWER, 0.0), true)
            sleep(10)
        }

        pedroMecanum.stop()

        val finalTimeSec = (System.currentTimeMillis() - startTime) / 1000.0
        val finalY = odometry.posY
        val totalDist = abs(finalY - startY)

        while (opModeIsActive()) {
            telemetry.addData("--- Lateral Tuner Results ---", "")
            telemetry.addData("Target Distance", "%.2f in", TARGET_DISTANCE_INCHES)
            telemetry.addData("Actual Distance", "%.2f in", totalDist)
            telemetry.addData("Overshoot/Error", "%.2f in", totalDist - TARGET_DISTANCE_INCHES)
            telemetry.addData("Elapsed Time", "%.2f sec", finalTimeSec)
            telemetry.addData("Max Strafe Velocity", "%.2f in/sec", maxVel)
            telemetry.update()
            sleep(50)
        }
    }
}
