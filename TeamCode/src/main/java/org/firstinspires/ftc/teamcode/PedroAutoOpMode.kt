package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
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

    private val poseFactory = PoseFactory.degrees()
    private val robot = RobotHardware()
    private lateinit var follower: Follower

    // Poses & Control Points
    private val startPose: Pose = poseFactory.of(56.0, 8.0, 90.0)
    private val shootingPose: Pose = poseFactory.of(56.0, 36.0, 90.0)
    private val reloadingStartPose: Pose = poseFactory.of(34.8, 11.0, 90.0)
    private val loadGardenPose: Pose = poseFactory.of(11.0, 14.2, 90.0)
    private val oppositeShootingPose: Pose = poseFactory.of(52.8, 126.9, 270.0)

    private val reloadingControl: Pose = poseFactory.of(56.0, 20.0, 90.0)
    private val gardenControl: Pose = poseFactory.of(18.0, 12.6, 90.0)
    private val oppositeShootingControl: Pose = poseFactory.of(18.0, 85.0, 90.0)

    // Path trajectory builders
    private fun startToShooting(): Path = line(startPose, shootingPose).constant(Math.toRadians(90.0))
    private fun shootingToReloading(): Path = curve(shootingPose, reloadingControl, reloadingStartPose).constant(Math.toRadians(90.0))
    private fun reloadingToGarden(): Path = curve(reloadingStartPose, gardenControl, loadGardenPose).constant(Math.toRadians(90.0))
    private fun gardenToOppositeShooting(): Path = curve(loadGardenPose, oppositeShootingControl, oppositeShootingPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    private var currentStep = "Initializing"

    private fun executeLaunchCommand(durationMs: Long = 5000L): NextCommand =
        series(
            instant {
                currentStep = "Launching elements for ${durationMs / 1000} seconds"
                robot.launcher?.velocity = IntakeLauncherSubsystem.TARGET_VELOCITY
                robot.launcher2?.velocity = IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY
            },
            delay(800L),
            instant {
                robot.leftLauncherServo?.power = 1.0
                robot.rightLauncherServo?.power = 1.0
                robot.intake?.power = 0.8
            },
            delay(durationMs - 800L),
            instant {
                robot.launcher?.velocity = 0.0
                robot.launcher2?.velocity = 0.0
                robot.intake?.power = 0.0
                robot.leftLauncherServo?.power = 0.0
                robot.rightLauncherServo?.power = 0.0
            }
        )

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

        follower.setPose(startPose)
        telemetry.addData("Status", "NextFTC Autonomous Ready")
        telemetry.update()
    }

    override fun onStartCommand(): NextCommand = series(
        instant { currentStep = "1: Following Path -> Shooting Position" },
        followPath(follower, startToShooting()),
        executeLaunchCommand(5000L),
        instant { currentStep = "2: Following Path -> Reloading Start" },
        followPath(follower, shootingToReloading()),
        instant { currentStep = "3: Following Path -> Load Garden" },
        followPath(follower, reloadingToGarden()),
        instant { currentStep = "4: Following Path -> Opposite Shooting" },
        followPath(follower, gardenToOppositeShooting()),
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
