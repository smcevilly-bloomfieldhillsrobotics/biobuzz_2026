package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.api.Paths
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.nextftc.*
import java.util.OptionalDouble

/**
 * A test Autonomous OpMode to drive from the audience wall to the center of the field (the Hive).
 * Built on the NextFTC framework.
 */
@Autonomous(name = "Test: Audience to Center Hive", group = "Test")
class CenterHiveTestAuto : NextOpMode() {

    private val robot = RobotHardware()
    private lateinit var follower: Follower
    private lateinit var telemetrySubsystem: TelemetrySubsystem

    private var currentStep = "Initializing"

    private val paths = PedroPaths()
    
    // Start at the center of the audience wall.
    private val startPose: Pose = paths.centerHiveStartPose

    // The trajectories to follow
    private val pathToIntermediate: Path = paths.centerHiveIntermediatePath
    private val pathToFinal: Path = paths.centerHiveFinalPath

    override fun onInit() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)
        robot.initPinpoint(hardwareMap)

        val pedroMecanum = robot.createPedroMecanum(hardwareMap)
        val pinpointLocalizer = PinpointLocalizer(hardwareMap, PinpointConfig { c ->
            c.name.set(PedroConstants.PINPOINT_DEVICE_NAME)
            c.xPodDirection.set(PedroConstants.X_POD_DIRECTION)
            c.yPodDirection.set(PedroConstants.Y_POD_DIRECTION)
            c.xPodOffset.set(PedroConstants.X_POD_OFFSET_MM)
            c.yPodOffset.set(PedroConstants.Y_POD_OFFSET_MM)
            c.offsetUnits.set(DistanceUnit.MM)
            if (PedroConstants.USE_CUSTOM_TICKS_PER_INCH) {
                c.ticksPerUnit.set(OptionalDouble.of(PedroConstants.CUSTOM_TICKS_PER_INCH))
                c.encoderResolutionUnit.set(DistanceUnit.INCH)
            }
        })

        follower = Follower(pinpointLocalizer, pedroMecanum, PedroAlgorithm())
            .withLogger { _ ->
                val pose = follower.pose()
                val packet = TelemetryPacket()
                packet.put("Step", currentStep)
                packet.put("X", pose.x())
                packet.put("Y", pose.y())
                packet.put("Heading (deg)", Math.toDegrees(pose.heading()))

                val fieldOverlay = packet.fieldOverlay()
                fieldOverlay.setStroke("#3B82F6") // Blue for test trajectory
                fieldOverlay.setStrokeWidth(2)
                
                // FTC Dashboard coordinates: (0,0) is center, +X goes UP, +Y goes LEFT
                // Pedro Pathing coordinates: (0,0) is bottom-left, +X goes UP, +Y goes RIGHT
                // We need to map Pedro (X, Y) to FTC Dashboard (-Y + 72, X - 72)
                val drawX = pose.x() - 72.0
                val drawY = 72.0 - pose.y() 
                val drawHeading = pose.heading() - Math.PI / 2.0 // Rotate heading by -90 deg for dashboard

                fieldOverlay.strokeCircle(drawX, drawY, 9.0)
                
                val length = 12.0
                fieldOverlay.strokeLine(
                    drawX,
                    drawY,
                    drawX + length * Math.cos(drawHeading),
                    drawY + length * Math.sin(drawHeading)
                )

                FtcDashboard.getInstance().sendTelemetryPacket(packet)
            }

        follower.setPose(startPose)
        telemetrySubsystem = TelemetrySubsystem(telemetry)
        
        telemetry.addData("Status", "Test Auto Ready")
        telemetry.addData("Start Pose", "(%.1f, %.1f)", startPose.x(), startPose.y())
        telemetry.update()
    }

    override fun onStartCommand(): NextCommand = series(
        instant { currentStep = "1: Driving to Intermediate Pose (Y=120)" },
        followPath(follower, pathToIntermediate),
        instant { currentStep = "2: Driving to Final Pose (X=52.8, Heading=270)" },
        followPath(follower, pathToFinal),
        LaunchRoutineCommand(robot, 5000L) { currentStep = "3: Launching elements for 5 seconds" },
        instant { currentStep = "Test Complete! Holding Position." }
    )

    override fun onLoop() {
        follower.update()
        val pose = follower.pose()
        
        val packet = TelemetryPacket()
        packet.put("Step", currentStep)
        packet.put("X", pose.x())
        packet.put("Y", pose.y())
        packet.put("Heading (deg)", Math.toDegrees(pose.heading()))
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
        
        telemetry.addData("--- Center Hive Test ---", "")
        telemetry.addData("Step", currentStep)
        telemetry.addData("X", pose.x())
        telemetry.addData("Y", pose.y())
        telemetry.addData("Heading", Math.toDegrees(pose.heading()))
        telemetry.update()
    }
}
