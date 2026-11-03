package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.nextftc.*
import java.util.OptionalDouble

/**
 * Autonomous OpMode for BioBuzz StarterBot built on NextFTC Framework (https://nextftc.dev/).
 */
@Autonomous(name = "BioBuzz Autonomous (NextFTC)", group = "Autonomous")
class PedroAutoOpMode : NextOpMode() {

    private val robot = RobotHardware()
    private val paths = PedroPaths()
    private lateinit var follower: Follower

    private var currentStep = "Initializing"



    override fun onInit() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)
        robot.initLimelight(hardwareMap)

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
            .withLogger { log ->
                val pose = follower.pose()
                val packet = TelemetryPacket()
                packet.put("Step", currentStep)
                packet.put("X", pose.x())
                packet.put("Y", pose.y())
                packet.put("Heading (deg)", Math.toDegrees(pose.heading()))

                val fieldOverlay = packet.fieldOverlay()
                fieldOverlay.setStroke("#22C55E")
                fieldOverlay.setStrokeWidth(2)
                fieldOverlay.strokeCircle(pose.x(), pose.y(), 9.0)

                val headingRad = pose.heading()
                val length = 12.0
                fieldOverlay.strokeLine(
                    pose.x(),
                    pose.y(),
                    pose.x() + length * Math.cos(headingRad),
                    pose.y() + length * Math.sin(headingRad)
                )

                FtcDashboard.getInstance().sendTelemetryPacket(packet)
            }

        follower.setPose(paths.startPose)
        telemetry.addData("Status", "NextFTC Autonomous Ready")
        telemetry.update()
    }

    override fun onStartCommand(): NextCommand = series(
        instant { currentStep = "1: Following Path -> Shooting Position" },
        followPath(follower, paths.shootingPositionPath),
        LaunchRoutineCommand(robot, 5000L) { currentStep = "Launching elements for 5 seconds" },
        instant { currentStep = "2: Following Path -> Reloading Start" },
        followPath(follower, paths.reloadingStartPath),
        instant { currentStep = "3: Following Path -> Load Garden" },
        followPath(follower, paths.loadGardenPath),
        instant { currentStep = "4: Following Path -> Opposite Shooting" },
        followPath(follower, paths.oppositeShootingPositionPath),
        instant { currentStep = "Autonomous Routine Complete!" }
    )

    override fun onLoop() {
        follower.update()
        val pose = follower.pose()
        telemetry.addData("--- NextFTC Autonomous ---", "")
        telemetry.addData("Step", currentStep)
        telemetry.addData("X", pose.x())
        telemetry.addData("Y", pose.y())
        telemetry.addData("Heading", Math.toDegrees(pose.heading()))
        telemetry.update()
    }
}
