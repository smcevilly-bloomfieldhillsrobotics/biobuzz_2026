package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.nextftc.*
import java.util.OptionalDouble
import kotlin.math.hypot

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

    // Start pose: 63, 9 (Red) or 81, 9 (Blue)
    protected open val isBlueAlliance: Boolean = false
    private val startPose: Pose get() = if (isBlueAlliance) paths.blueAprilTagStartPose else paths.aprilTagStartPose
    
    // Command and timing states
    private var autoStartTimeMs = 0L
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
            autoStartTimeMs = System.currentTimeMillis()
            currentStep = "Searching for AprilTag for 5 seconds..."
            aprilTagSearchStartTime = System.currentTimeMillis()
            aprilTagFound = false
        },
        // Wait up to 5 seconds (or until 10s left in match), continuously checking for an AprilTag
        object : NextCommand {
            override fun start() {}
            
            override fun execute() {
                val target = aimingSubsystem.getBestAprilTag()
                if (target != null && target.isValid && target.id in validTagIds) {
                    aprilTagFound = true
                }
            }
            
            override fun isFinished(): Boolean {
                val elapsed = System.currentTimeMillis() - autoStartTimeMs
                val timeToPark = elapsed >= 20000L // 30s autonomous period - 10s remaining
                return (System.currentTimeMillis() - aprilTagSearchStartTime) >= 5000 || aprilTagFound || timeToPark
            }
            
            override fun end(interrupted: Boolean) {}
        },
        // Conditional branch based on if tag was found (finishes when 10 seconds remain in match)
        object : NextCommand {
            private var activeCommand: NextCommand? = null
            private var started = false

            override fun start() {
                activeCommand = if (aprilTagFound) {
                    series(
                        instant { currentStep = "AprilTag found! Aiming toward Hive Cell..." },
                        followPath(follower, if (isBlueAlliance) paths.blueStartAimPath else paths.redStartAimPath, timeoutMs = 1500L),
                        instant { currentStep = "Centered! Launching elements." },
                        LaunchRoutineCommand(
                            robot = robot,
                            durationMs = 5000L,
                            tagValidator = {
                                val t = aimingSubsystem.getBestAprilTag()
                                t != null && t.isValid && t.id in validTagIds
                            },
                            onStatusUpdate = { status -> currentStep = status },
                            onStart = { currentStep = "Spinning up flywheels (Checking AprilTag)..." }
                        ),
                        instant { currentStep = "Re-aligning heading for smooth park..." },
                        followPath(follower, if (isBlueAlliance) paths.blueStartRealignPath else paths.redStartRealignPath, timeoutMs = 1500L)
                    )
                } else {
                    series(
                        instant { currentStep = "No AprilTag found. Driving to intermediate pose." },
                        followPath(follower, if (isBlueAlliance) paths.blueAprilTagIntermediatePath else paths.aprilTagIntermediatePath),
                        instant { currentStep = "Driving to opposite side." },
                        followPath(follower, if (isBlueAlliance) paths.blueAprilTagFinalPath else paths.aprilTagFinalPath),
                        instant { currentStep = "Opposite side reached. Aiming toward Hive Cell..." },
                        followPath(follower, if (isBlueAlliance) paths.blueOppositeAimPath else paths.redOppositeAimPath, timeoutMs = 1500L),
                        instant { currentStep = "Centered! Launching elements." },
                        LaunchRoutineCommand(
                            robot = robot,
                            durationMs = 5000L,
                            tagValidator = {
                                val t = aimingSubsystem.getBestAprilTag()
                                t != null && t.isValid && t.id in validTagIds
                            },
                            onStatusUpdate = { status -> currentStep = status },
                            onStart = { currentStep = "Spinning up flywheels (Checking AprilTag)..." }
                        ),
                        instant { currentStep = "Re-aligning heading for smooth park..." },
                        followPath(follower, if (isBlueAlliance) paths.blueOppositeRealignPath else paths.redOppositeRealignPath, timeoutMs = 1500L)
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
                val elapsed = System.currentTimeMillis() - autoStartTimeMs
                val tenSecondsLeft = elapsed >= 20000L // 20s elapsed into 30s auto
                return (started && (activeCommand?.isFinished() ?: true)) || tenSecondsLeft
            }

            override fun end(interrupted: Boolean) {
                activeCommand?.end(interrupted)
            }
        },
        instant { currentStep = "Moving to Park End Pose..." },
        object : NextCommand {
            private var followCmd: NextCommand? = null

            override fun start() {
                val currentPose = follower.pose()
                val path = paths.pathToPark(currentPose, isBlueAlliance)
                followCmd = followPath(follower, path)
                followCmd?.start()
            }

            override fun execute() {
                followCmd?.execute()
            }

            override fun isFinished(): Boolean = followCmd?.isFinished() ?: true

            override fun end(interrupted: Boolean) {
                followCmd?.end(interrupted)
            }
        },
        instant { currentStep = "Park Complete! Autonomous Routine Complete!" }
    )

    override fun onLoop() {
        follower.update()
        val pose = follower.pose()
        val target = aimingSubsystem.getBestAprilTag()
        if (target != null && target.isValid && target.id in validTagIds) {
            aprilTagFound = true
        }

        // Periodic Limelight vision re-localization: if an upright tag cluster is in view,
        // softly fuse/correct odometry drift into the Pedro Follower pose
        val visionPose = aimingSubsystem.estimateRobotPose(pose.heading())
        if (visionPose != null) {
            // Apply correction if odometry error exceeds 1.5 inches but is within 10 inches (reject false spikes)
            val odometryError = hypot(visionPose.x - pose.x(), visionPose.y - pose.y())
            if (odometryError in 1.5..10.0) {
                // Smooth fusion (80% odometry, 20% vision correction) to eliminate abrupt jumping
                val correctedX = pose.x() * 0.80 + visionPose.x * 0.20
                val correctedY = pose.y() * 0.80 + visionPose.y * 0.20
                follower.setPose(PoseFactory.degrees().of(correctedX, correctedY, Math.toDegrees(pose.heading())))
            }
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