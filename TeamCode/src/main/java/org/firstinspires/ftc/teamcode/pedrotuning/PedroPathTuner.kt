package org.firstinspires.ftc.teamcode.pedrotuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.paths.Path
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.OdometrySubsystem
import org.firstinspires.ftc.teamcode.RobotHardware
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Pedro Pathing Tuning Step 5: Closed-Loop Path PIDF & Follower Feedback Tuning.
 *
 * Continuously loops a 24" x 24" test box path while allowing live gain adjustments via FTC Dashboard:
 * - TRANSLATIONAL_P / TRANSLATIONAL_D
 * - HEADING_P / HEADING_D
 * - MAX_POWER
 */
@Config
@Autonomous(name = "5. Pedro Path Feedback Tuner", group = "Pedro Tuning")
class PedroPathTuner : LinearOpMode() {

    companion object {
        @JvmField var TRANSLATIONAL_P = 0.06
        @JvmField var TRANSLATIONAL_D = 0.005
        @JvmField var HEADING_P = 0.70
        @JvmField var HEADING_D = 0.04
        @JvmField var MAX_POWER = 0.50
        @JvmField var BOX_SIZE_INCHES = 24.0
    }

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var odometry: OdometrySubsystem

    private var lastErrorX = 0.0
    private var lastErrorY = 0.0
    private var lastErrorHeading = 0.0
    private var lastTime = System.currentTimeMillis()

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initPinpoint(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)
        odometry = OdometrySubsystem(robot)

        odometry.resetPosition(0.0, 0.0, 0.0)

        val poseFactory = PoseFactory.degrees()
        val p0 = poseFactory.of(0.0, 0.0, 0.0)
        val p1 = poseFactory.of(BOX_SIZE_INCHES, 0.0, 0.0)
        val p2 = poseFactory.of(BOX_SIZE_INCHES, BOX_SIZE_INCHES, 90.0)
        val p3 = poseFactory.of(0.0, BOX_SIZE_INCHES, 180.0)

        val boxPaths = arrayOf(
            Paths.line(p0, p1).linear(p0, p1),
            Paths.line(p1, p2).linear(p1, p2),
            Paths.line(p2, p3).linear(p2, p3),
            Paths.line(p3, p0).linear(p3, p0)
        )

        telemetry.addData("Status", "Ready. Tune gains live in FTC Dashboard @Config.")
        telemetry.update()

        waitForStart()

        var currentPathIdx = 0

        while (opModeIsActive()) {
            val path = boxPaths[currentPathIdx]
            val completed = followPathStep(path)

            if (completed) {
                currentPathIdx = (currentPathIdx + 1) % boxPaths.size
                sleep(200)
            }
        }

        pedroMecanum.stop()
    }

    private fun followPathStep(path: Path): Boolean {
        val targetPose = path.endPose()
        odometry.update(null)

        val currentX = odometry.posX
        val currentY = odometry.posY
        val currentHeadingRad = Math.toRadians(odometry.heading)

        val dx = targetPose.x() - currentX
        val dy = targetPose.y() - currentY
        val distanceRemaining = hypot(dx, dy)

        var headingError = targetPose.heading() - currentHeadingRad
        while (headingError > Math.PI) headingError -= 2 * Math.PI
        while (headingError < -Math.PI) headingError += 2 * Math.PI

        if (distanceRemaining < 1.0 && abs(Math.toDegrees(headingError)) < 2.0) {
            pedroMecanum.stop()
            return true
        }

        val now = System.currentTimeMillis()
        val dt = ((now - lastTime) / 1000.0).coerceAtLeast(0.001)

        val dX = (dx - lastErrorX) / dt
        val dY = (dy - lastErrorY) / dt
        val dHeading = (headingError - lastErrorHeading) / dt

        lastErrorX = dx
        lastErrorY = dy
        lastErrorHeading = headingError
        lastTime = now

        val fieldXCmd = (dx * TRANSLATIONAL_P + dX * TRANSLATIONAL_D).coerceIn(-MAX_POWER, MAX_POWER)
        val fieldYCmd = (dy * TRANSLATIONAL_P + dY * TRANSLATIONAL_D).coerceIn(-MAX_POWER, MAX_POWER)
        val turnPower = (headingError * HEADING_P + dHeading * HEADING_D).coerceIn(-MAX_POWER, MAX_POWER)

        val cosH = cos(currentHeadingRad)
        val sinH = sin(currentHeadingRad)

        val robotForward = fieldXCmd * cosH + fieldYCmd * sinH
        val robotStrafe = -fieldXCmd * sinH + fieldYCmd * cosH

        pedroMecanum.drive(DrivePowers(robotForward, robotStrafe, turnPower), true)

        val packet = TelemetryPacket()
        packet.put("Target X", targetPose.x())
        packet.put("Current X", currentX)
        packet.put("Target Y", targetPose.y())
        packet.put("Current Y", currentY)
        packet.put("X Error", dx)
        packet.put("Y Error", dy)
        packet.put("Distance Error", distanceRemaining)
        packet.put("Heading Error (deg)", Math.toDegrees(headingError))

        val overlay = packet.fieldOverlay()
        overlay.setStroke("#4CAF50")
        overlay.strokeCircle(currentX, currentY, 9.0)
        overlay.setStroke("#F44336")
        overlay.strokeCircle(targetPose.x(), targetPose.y(), 3.0)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        telemetry.addData("--- Pedro Path Feedback Tuner ---", "")
        telemetry.addData("Target Pose", "X: %.1f, Y: %.1f, Head: %.1f°", targetPose.x(), targetPose.y(), Math.toDegrees(targetPose.heading()))
        telemetry.addData("Current Pose", "X: %.1f, Y: %.1f, Head: %.1f°", currentX, currentY, odometry.heading)
        telemetry.addData("Distance Error", "%.2f in", distanceRemaining)
        telemetry.addData("Heading Error", "%.2f deg", Math.toDegrees(headingError))
        telemetry.update()

        return false
    }
}
