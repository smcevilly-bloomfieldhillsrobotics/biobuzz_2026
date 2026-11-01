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

/**
 * Pedro Pathing Tuning Step 4: Turn & Angular Kinematics Tuning.
 *
 * Rotates the robot in place a specified angle (default: 360.0 degrees) at TURN_POWER.
 * Measures max angular velocity (deg/sec) and angular response.
 */
@Config
@Autonomous(name = "4. Pedro Turn Tuner", group = "Pedro Tuning")
class PedroTurnTuner : LinearOpMode() {

    companion object {
        @JvmField var TARGET_ANGLE_DEGREES = 360.0
        @JvmField var TURN_POWER = 0.40
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

        telemetry.addData("Status", "Ready. Clear 360° space around robot.")
        telemetry.addData("Target Angle", "%.1f degrees", TARGET_ANGLE_DEGREES)
        telemetry.update()

        waitForStart()

        if (!opModeIsActive()) return

        val startHeading = odometry.heading
        val startTime = System.currentTimeMillis()

        var accumulatedAngle = 0.0
        var lastHeading = startHeading
        var maxAngularVel = 0.0
        var lastTime = startTime

        while (opModeIsActive()) {
            odometry.update(null)

            val currentHeading = odometry.heading
            var dHeading = currentHeading - lastHeading
            while (dHeading > 180.0) dHeading -= 360.0
            while (dHeading < -180.0) dHeading += 360.0

            accumulatedAngle += dHeading
            lastHeading = currentHeading

            val now = System.currentTimeMillis()
            val dt = (now - lastTime) / 1000.0
            if (dt > 0.005) {
                val instAngVel = abs(dHeading) / dt
                if (instAngVel > maxAngularVel) maxAngularVel = instAngVel
                lastTime = now
            }

            val packet = TelemetryPacket()
            packet.put("Accumulated Heading (deg)", accumulatedAngle)
            packet.put("Target Heading (deg)", TARGET_ANGLE_DEGREES)
            packet.put("Max Angular Velocity (deg/s)", maxAngularVel)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            if (abs(accumulatedAngle) >= abs(TARGET_ANGLE_DEGREES)) {
                break
            }

            // Turn Counter-Clockwise (positive turn)
            pedroMecanum.drive(DrivePowers(0.0, 0.0, TURN_POWER), true)
            sleep(10)
        }

        pedroMecanum.stop()

        val finalTimeSec = (System.currentTimeMillis() - startTime) / 1000.0

        while (opModeIsActive()) {
            telemetry.addData("--- Turn Tuner Results ---", "")
            telemetry.addData("Target Angle", "%.2f deg", TARGET_ANGLE_DEGREES)
            telemetry.addData("Actual Angle", "%.2f deg", accumulatedAngle)
            telemetry.addData("Overshoot/Error", "%.2f deg", abs(accumulatedAngle) - abs(TARGET_ANGLE_DEGREES))
            telemetry.addData("Elapsed Time", "%.2f sec", finalTimeSec)
            telemetry.addData("Max Angular Velocity", "%.2f deg/sec", maxAngularVel)
            telemetry.update()
            sleep(50)
        }
    }
}
