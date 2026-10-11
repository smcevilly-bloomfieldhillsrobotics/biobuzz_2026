package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

/**
 * Sample Autonomous using Road Runner with Tank Drive and Pinpoint Odometry.
 */
@Autonomous(name = "BioBuzz RoadRunner Tank Auto", group = "Autonomous")
class RoadRunnerAutoOpMode : LinearOpMode() {

    private val robot = RobotHardware()
    private lateinit var drive: RoadRunnerTankDrive

    override fun runOpMode() {
        robot.init(hardwareMap)
        drive = RoadRunnerTankDrive(robot)

        telemetry.addData("Status", "Road Runner Tank Initialized")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val pose = drive.updatePose()

            val packet = TelemetryPacket()
            packet.put("RR X (in)", pose.position.x)
            packet.put("RR Y (in)", pose.position.y)
            packet.put("RR Heading (rad)", pose.heading.toDouble())

            val fieldOverlay = packet.fieldOverlay()
            fieldOverlay.setStroke("#4CAF50")
            fieldOverlay.strokeCircle(pose.position.x, pose.position.y, 9.0)

            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            telemetry.addData("X", "%.2f", pose.position.x)
            telemetry.addData("Y", "%.2f", pose.position.y)
            telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading.toDouble()))
            telemetry.update()
        }

        drive.stop()
    }
}
