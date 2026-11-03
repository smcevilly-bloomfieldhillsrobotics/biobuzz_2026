package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.algorithm.Algorithm
import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.api.PoseFactory
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.commands.Commands.conditional
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathTracker
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.OptionalDouble

/**
 * Autonomous OpMode for BioBuzz StarterBot using Pedro Pathing 3.0, native PinpointLocalizer,
 * and Ivy Command Scheduler.
 */
@Autonomous(name = "BioBuzz Autonomous (Pedro Pathing)", group = "Autonomous")
class PedroAutoOpMode : OpMode() {

    private val poseFactory = PoseFactory.degrees()
    private val robot = RobotHardware()
    private lateinit var follower: Follower

    /**
     * Native Pedro Pathing Ivy follow command wrapper.
     */
    private fun follow(follower: Follower, path: Path): Command =
        Command.build()
            .setStart {
                follower.follow(path)
                follower.update()
            }
            .setExecute { follower.update() }
            .setDone { !follower.isBusy() }
            .setEnd { interrupted ->
                if (interrupted == EndCondition.INTERRUPTED) {
                    follower.stop()
                }
            }

    // Poses (Adjusted with 9" robot radius clearance from X=0 wall)
    private val startPose: Pose = poseFactory.of(56.0, 8.0, 90.0)
    private val shootingPose: Pose = poseFactory.of(56.0, 36.0, 90.0)
    private val reloadingStartPose: Pose = poseFactory.of(34.8, 11.0, 90.0)
    private val loadGardenPose: Pose = poseFactory.of(11.0, 14.2, 90.0) // X=11.0 leaves 2" clearance from X=0 wall
    private val oppositeShootingPose: Pose = poseFactory.of(52.8, 126.9, 270.0)

    // Control Points
    private val reloadingControl: Pose = poseFactory.of(56.0, 20.0, 90.0)
    private val gardenControl: Pose = poseFactory.of(18.0, 12.6, 90.0)
    private val oppositeShootingControl: Pose = poseFactory.of(18.0, 85.0, 90.0) // X=18.0 leaves 9" safe clearance from X=0 wall

    // Path methods
    private fun startToShooting(): Path =
        line(startPose, shootingPose).constant(Math.toRadians(90.0))

    private fun shootingToReloading(): Path =
        curve(shootingPose, reloadingControl, reloadingStartPose).constant(Math.toRadians(90.0))

    private fun reloadingToGarden(): Path =
        curve(reloadingStartPose, gardenControl, loadGardenPose).constant(Math.toRadians(90.0))

    private fun gardenToOppositeShooting(): Path =
        curve(loadGardenPose, oppositeShootingControl, oppositeShootingPose)
            .linear(Math.toRadians(90.0), Math.toRadians(270.0))

    private var currentStep = "Initializing"

    private fun executeLaunchCommand(launchDurationMs: Long): Command {
        var startTimeMs = 0L
        var flywheelsReady = false
        var isDone = false

        val minVel1 = IntakeLauncherSubsystem.MIN_VELOCITY
        val minVel2 = IntakeLauncherSubsystem.LAUNCHER_2_MIN_VELOCITY

        return Command.build()
            .setStart {
                currentStep = "AprilTag Detected! Launching for ${launchDurationMs / 1000} seconds"
                startTimeMs = System.currentTimeMillis()
                flywheelsReady = false
                isDone = false
                robot.launcher?.velocity = IntakeLauncherSubsystem.TARGET_VELOCITY
                robot.launcher2?.velocity = IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY
            }
            .setExecute {
                val now = System.currentTimeMillis()
                val v1 = robot.launcher?.velocity ?: 0.0
                val v2 = robot.launcher2?.velocity ?: 0.0

                if (!flywheelsReady && ((v1 > minVel1 && v2 > minVel2) || (now - startTimeMs) > 800)) {
                    flywheelsReady = true
                }

                if (flywheelsReady) {
                    robot.leftLauncherServo?.power = 1.0
                    robot.rightLauncherServo?.power = 1.0
                    robot.intake?.power = 0.8
                } else {
                    robot.leftLauncherServo?.power = 0.0
                    robot.rightLauncherServo?.power = 0.0
                }

                if (now - startTimeMs >= launchDurationMs) {
                    isDone = true
                }
            }
            .setDone { isDone }
            .setEnd {
                robot.launcher?.velocity = 0.0
                robot.launcher2?.velocity = 0.0
                robot.intake?.power = 0.0
                robot.leftLauncherServo?.power = 0.0
                robot.rightLauncherServo?.power = 0.0
            }
    }

    private fun autoRoutine(): Command {
        var aprilTagFound = false
        var tagWaitStart = 0L

        return sequential(
            // 1. Wait up to 5s for AprilTag
            Command.build()
                .setStart {
                    currentStep = "Waiting for AprilTag (up to 5s)"
                    tagWaitStart = System.currentTimeMillis()
                    aprilTagFound = false
                }
                .setExecute {
                    val tag = getAprilTagTarget()
                    if (tag != null && tag.isValid) {
                        aprilTagFound = true
                    }
                }
                .setDone { aprilTagFound || (System.currentTimeMillis() - tagWaitStart) >= 5000L },

            // 2. Launch Sequence if AprilTag was detected
            conditional(
                { aprilTagFound },
                executeLaunchCommand(5000L),
                instant { currentStep = "No AprilTag detected within 5s - Proceeding to paths" }
            ),

            // 3. Traversal Sequence using Pedro Pathing native Follower
            follow(follower, startToShooting()),
            follow(follower, shootingToReloading()),
            follow(follower, reloadingToGarden()),
            follow(follower, gardenToOppositeShooting()),

            instant {
                currentStep = "Complete! Holding final position"
            }
        )
    }

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        Scheduler.reset()

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
        follower.setPose(startPose)

        telemetry.addData("Status", "Pedro Pathing Autonomous Initialized")
        telemetry.update()
    }

    override fun start() {
        schedule(autoRoutine())
    }

    override fun loop() {
        follower.update()
        Scheduler.execute()

        val pose = follower.pose()

        val packet = TelemetryPacket()
        packet.put("Step", currentStep)
        packet.put("X", pose.x())
        packet.put("Y", pose.y())
        packet.put("Heading (deg)", Math.toDegrees(pose.heading()))

        // Draw Pedro Pathing Robot Overlay on FTC Dashboard Canvas
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

        telemetry.addData("--- Pedro Autonomous Status ---", "")
        telemetry.addData("Step", currentStep)
        telemetry.addData("X", pose.x())
        telemetry.addData("Y", pose.y())
        telemetry.addData("Heading", Math.toDegrees(pose.heading()))
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
                val distInches = if (botpose != null) Math.hypot(botpose.position.x, botpose.position.y) * 39.3701 else 0.0
                return AprilTagTarget(id = -1, txDegrees = result.tx, distanceInches = distInches, isValid = true)
            }
            return null
        }

        var bestTag: AprilTagTarget? = null
        var minDistance = Double.MAX_VALUE

        for (fr in fiducials) {
            val pose = fr.targetPoseCameraSpace
            val distInches = if (pose != null) Math.sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z) * 39.3701 else 0.0

            if (distInches > 0.0 && distInches < minDistance) {
                minDistance = distInches
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            } else if (bestTag == null) {
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            }
        }

        return bestTag
    }
}

class PedroAlgorithm : Algorithm {
    private var isBusy = false
    private var currentPose = Pose.zero()

    override fun reset() {
        isBusy = true
    }

    override fun isBusy(): Boolean = isBusy

    override fun calculatePath(
        drivetrain: Drivetrain,
        pathTracker: PathTracker,
        motionState: MotionState,
        dt: Double
    ): DrivePowers {
        val robotPose = motionState.pose()
        currentPose = robotPose

        val endPose = pathTracker.endPose()
        val dx = endPose.x() - robotPose.x()
        val dy = endPose.y() - robotPose.y()
        var headingError = endPose.heading() - robotPose.heading()
        while (headingError > Math.PI) headingError -= 2 * Math.PI
        while (headingError < -Math.PI) headingError += 2 * Math.PI

        if (Math.hypot(dx, dy) < 2.0 && Math.abs(Math.toDegrees(headingError)) < 4.0) {
            isBusy = false
            return DrivePowers.zero()
        }

        val cosH = Math.cos(robotPose.heading())
        val sinH = Math.sin(robotPose.heading())

        val fwdCmd = (dx * cosH + dy * sinH) * PedroConstants.TRANSLATIONAL_P
        val strCmd = (-dx * sinH + dy * cosH) * PedroConstants.TRANSLATIONAL_P
        val turnCmd = headingError * PedroConstants.HEADING_P

        return DrivePowers(
            fwdCmd.coerceIn(-PedroConstants.MAX_AUTO_POWER, PedroConstants.MAX_AUTO_POWER),
            strCmd.coerceIn(-PedroConstants.MAX_AUTO_POWER, PedroConstants.MAX_AUTO_POWER),
            turnCmd.coerceIn(-0.4, 0.4)
        )
    }

    override fun calculateHold(
        drivetrain: Drivetrain,
        targetPose: Pose,
        motionState: MotionState,
        useHoldScaling: Boolean,
        dt: Double
    ): DrivePowers = DrivePowers.zero()

    override fun completion(): Double = 1.0
    override fun closestPose(): Pose = currentPose
    override fun closestTangent(): Vector2D = Vector2D.zero()
    override fun closestNormal(): Vector2D = Vector2D.zero()
    override fun curvature(): Double = 0.0
    override fun remainingDistance(): Double = 0.0
    override fun atParametricEnd(): Boolean = true
    override fun debug(): Map<String, Any> = emptyMap()
}
