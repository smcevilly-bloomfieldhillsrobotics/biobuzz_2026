package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.nextftc.*
import java.util.OptionalDouble

/**
 * Base autonomous OpMode that checks for an AprilTag for the first 5 seconds.
 * If seen and it's within the valid range, it launches. 
 * If not seen or out of range, it drives to the opposite side and launches.
 */
abstract class AprilTagBaseAutoOpMode : NextOpMode() {

    private val robot = RobotHardware()
    private lateinit var follower: Follower
    private lateinit var aimingSubsystem: LimelightAimingSubsystem
    private lateinit var intakeLauncher: IntakeLauncherSubsystem
    private lateinit var telemetrySubsystem: TelemetrySubsystem

    private var currentStep = "Initializing"
    
    private val paths = PedroPaths()

    // Start pose: 72, 9
    private val startPose: Pose = paths.aprilTagStartPose
    
    // Command states
    private var aprilTagSearchStartTime = 0L
    private var aprilTagFound = false
    
    /**
     * Define the valid range of AprilTag IDs for this OpMode.
     */
    abstract val validTagIds: IntRange

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
            .withLogger { _ ->
                val pose = follower.pose()
                val packet = TelemetryPacket()
                packet.put("Step", currentStep)
                packet.put("X", pose.x())
                packet.put("Y", pose.y())
                packet.put("Heading (deg)", Math.toDegrees(pose.heading()))
                
                val fieldOverlay = packet.fieldOverlay()
                fieldOverlay.setStroke("#F59E0B")
                fieldOverlay.setStrokeWidth(2)
                
                val drawX = pose.x() - 72.0
                val drawY = 72.0 - pose.y() 
                val drawHeading = pose.heading() - Math.PI / 2.0 

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

        aimingSubsystem = LimelightAimingSubsystem(hardwareMap)
        intakeLauncher = IntakeLauncherSubsystem(robot)
        telemetrySubsystem = TelemetrySubsystem(telemetry)

        follower.setPose(startPose)
        telemetry.addData("Status", "AprilTag Autonomous Ready")
        telemetry.update()
    }

    override fun onStartCommand(): NextCommand = series(
        instant { 
            currentStep = "Searching for AprilTag for 5 seconds..."
            aprilTagSearchStartTime = System.currentTimeMillis()
            aprilTagFound = false
        },
        // Wait 5 seconds, continuously checking for an AprilTag
        object : NextCommand {
            override fun start() {}
            
            override fun execute() {
                val target = aimingSubsystem.getBestAprilTag()
                if (target != null && target.isValid && target.id in validTagIds) {
                    aprilTagFound = true
                }
            }
            
            override fun isFinished(): Boolean {
                return (System.currentTimeMillis() - aprilTagSearchStartTime) >= 5000 || aprilTagFound
            }
            
            override fun end(interrupted: Boolean) {}
        },
        // Conditional branch based on if tag was found
        object : NextCommand {
            private var activeCommand: NextCommand? = null
            private var started = false

            override fun start() {
                activeCommand = if (aprilTagFound) {
                    series(
                        instant { currentStep = "AprilTag found! Preparing to launch." },
                        LaunchRoutineCommand(
                            robot = robot,
                            durationMs = -1L,
                            tagValidator = {
                                val t = aimingSubsystem.getBestAprilTag()
                                t != null && t.isValid && t.id in validTagIds
                            },
                            onStatusUpdate = { status -> currentStep = status },
                            onStart = { currentStep = "Spinning up flywheels (Checking AprilTag)..." }
                        )
                    )
                } else {
                    series(
                        instant { currentStep = "No AprilTag found. Driving to intermediate pose." },
                        followPath(follower, paths.aprilTagIntermediatePath),
                        instant { currentStep = "Driving to opposite side." },
                        followPath(follower, paths.aprilTagFinalPath),
                        instant { currentStep = "Opposite side reached." },
                        LaunchRoutineCommand(
                            robot = robot,
                            durationMs = -1L,
                            tagValidator = {
                                val t = aimingSubsystem.getBestAprilTag()
                                t != null && t.isValid && t.id in validTagIds
                            },
                            onStatusUpdate = { status -> currentStep = status },
                            onStart = { currentStep = "Spinning up flywheels (Checking AprilTag)..." }
                        )
                    )
                }
                activeCommand?.start()
                started = true
            }

            override fun execute() {
                if (started) {
                    activeCommand?.execute()
                }
            }

            override fun isFinished(): Boolean {
                return started && (activeCommand?.isFinished() ?: true)
            }

            override fun end(interrupted: Boolean) {
                activeCommand?.end(interrupted)
            }
        },
        instant { currentStep = "Autonomous Routine Complete!" }
    )

    override fun onLoop() {
        follower.update()
        val pose = follower.pose()
        val target = aimingSubsystem.getBestAprilTag()
        if (target != null && target.isValid && target.id in validTagIds) {
            aprilTagFound = true
        }

        telemetrySubsystem.publishAutoData(
            opModeName = "AprilTag Autonomous",
            currentStep = currentStep,
            intakeLauncher = intakeLauncher,
            poseX = pose.x(),
            poseY = pose.y(),
            headingDeg = Math.toDegrees(pose.heading()),
            aprilTag = target,
            aprilTagFound = aprilTagFound,
            nectarLauncherVel = robot.nectarLauncher?.velocity ?: 0.0,
            pollenLauncherVel = robot.pollenLauncher?.velocity ?: 0.0
        )
    }
}