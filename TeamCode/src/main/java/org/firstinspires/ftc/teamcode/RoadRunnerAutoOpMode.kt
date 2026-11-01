package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Autonomous OpMode for BioBuzz StarterBot (Tank Drive) using goBILDA Pinpoint Odometry,
 * RoadRunner 1.0 trajectory following (with waypoint fallback), Limelight 3A AprilTag targeting,
 * and launcher sequencing before initiating path navigation.
 */
@Autonomous(name = "BioBuzz Autonomous (Drive to Target)", group = "Autonomous")
class RoadRunnerAutoOpMode : LinearOpMode() {

    companion object {
        const val DESIRED_TAG_DISTANCE_INCHES = 24.0 // Target distance from robot camera to AprilTag
        const val METERS_TO_INCHES = 39.3701
        const val SPEED_P = 0.025                    // Power per inch of range error
        const val TURN_P = 0.015                     // Power per degree of bearing/tx error
        const val MAX_APPROACH_SPEED = 0.40          // Max forward speed during tag approach
        const val MAX_APPROACH_TURN = 0.30           // Max turning speed during tag approach
    }

    private val robot = RobotHardware()
    private lateinit var drive: RoadRunnerTankDrive
    private var tankDrive: TankDrive? = null
    private var paths: RoadRunnerPaths? = null
    private var currentStep = "Initializing"

    override fun runOpMode() {
        // Direct telemetry to both Driver Station and FTC Dashboard
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.init(hardwareMap)
        drive = RoadRunnerTankDrive(robot)

        // Try initializing TankDrive for RoadRunner trajectory action execution
        try {
            tankDrive = TankDrive(hardwareMap, Pose2d(56.0, 8.0, Math.toRadians(90.0)))
            paths = RoadRunnerPaths(tankDrive!!)
        } catch (_: Exception) {
            tankDrive = null
            paths = null
        }

        // Reset starting odometry pose to initial field position (56.0, 8.0, 90.0 deg) defined in RoadRunnerPaths
        robot.pinpoint?.position = Pose2D(
            DistanceUnit.INCH,
            56.0,
            8.0,
            AngleUnit.DEGREES,
            90.0
        )

        currentStep = "Ready (Waiting for Start)"
        val initialPose = drive.updatePose()
        updateAutoTelemetry(initialPose.position.x, initialPose.position.y, Math.toDegrees(initialPose.heading.toDouble()))

        waitForStart()

        if (!opModeIsActive()) return

        // --- Step 0: Execute Launcher Function BEFORE initiating drive ---
        currentStep = "0: Launching game elements"
        updatePoseAndTelemetry()
        executeLaunchSequence(launchDurationMs = 2500)
        sleep(200)

        val rrPaths = paths
        val rrTank = tankDrive

        if ((rrPaths != null) && (rrTank != null)) {
            // Path 1: Drive to Shooting Position (56.0, 36.0, 90°)
            currentStep = "1: Following Path -> Shooting Position (56.0, 36.0)"
            updatePoseAndTelemetry()
            runBlockingAction(rrPaths.shootingPositionPath())
            sleep(300)

            // Optional: Tag alignment & approach at shooting position
            currentStep = "Limelight AprilTag Align & Approach"
            approachAprilTagIfDetected(targetDistanceInches = DESIRED_TAG_DISTANCE_INCHES, timeoutSec = 3.0)
            sleep(200)

            // Path 2: Drive to Reloading Start (34.80, 10.97, 180°)
            currentStep = "2: Following Path -> Reloading Start (34.80, 10.97)"
            updatePoseAndTelemetry()
            runBlockingAction(rrPaths.reloadingStartPath())
            sleep(300)

            // Path 3: Drive reverse to Load Garden (0.83, 14.19, -5.42°)
            currentStep = "3: Following Path -> Load Garden (0.83, 14.19)"
            updatePoseAndTelemetry()
            runBlockingAction(rrPaths.loadGardenPath())
            sleep(300)

            // Path 4: Drive to Opposite Shooting Position (52.82, 126.89, 270°)
            currentStep = "4: Following Path -> Opposite Shooting (52.82, 126.89)"
            updatePoseAndTelemetry()
            runBlockingAction(rrPaths.oppositeShootingPositionPath())
            sleep(300)

        } else {
            // Fallback waypoint navigation directly visiting each pose defined in RoadRunnerPaths:
            // 1) shootingPosition: (56.0, 36.0)
            // 2) reloadingStart: (34.8008, 10.97)
            // 3) loadGarden: (0.8333, 14.1918)
            // 4) oppositeShootingPosition: (52.8239, 126.8941)

            currentStep = "1: Moving to Shooting Position (56.0, 36.0)"
            driveToPoint(targetX = 56.0, targetY = 36.0, speed = 0.5)
            turnToHeading(targetAngleDegrees = 90.0, speed = 0.4)
            sleep(300)

            currentStep = "Limelight AprilTag Align & Approach"
            approachAprilTagIfDetected(targetDistanceInches = DESIRED_TAG_DISTANCE_INCHES, timeoutSec = 3.0)
            sleep(200)

            currentStep = "2: Moving to Reloading Start (34.80, 10.97)"
            driveToPoint(targetX = 34.8008, targetY = 10.97, speed = 0.5)
            turnToHeading(targetAngleDegrees = 180.0, speed = 0.4)
            sleep(300)

            currentStep = "3: Moving to Load Garden (0.83, 14.19)"
            driveToPoint(targetX = 0.8333, targetY = 14.1918, speed = 0.5)
            turnToHeading(targetAngleDegrees = -5.4183, speed = 0.4)
            sleep(300)

            currentStep = "4: Moving to Opposite Shooting (52.82, 126.89)"
            driveToPoint(targetX = 52.8239, targetY = 126.8941, speed = 0.5)
            turnToHeading(targetAngleDegrees = 270.0, speed = 0.4)
            sleep(300)
        }

        currentStep = "Complete! Holding final position"

        // Keep updating telemetry & dashboard while active
        while (opModeIsActive()) {
            val currentPose = drive.updatePose()
            updateAutoTelemetry(currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()))
            sleep(20)
        }

        drive.stop()
    }

    /**
     * Spins up the dual launcher flywheel motors, feeds elements via the windmill servo and intake,
     * and stops all launcher motors after firing.
     */
    private fun executeLaunchSequence(launchDurationMs: Long = 2500) {
        val targetVel1 = IntakeLauncherSubsystem.TARGET_VELOCITY
        val minVel1 = IntakeLauncherSubsystem.MIN_VELOCITY
        val targetVel2 = IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY
        val minVel2 = IntakeLauncherSubsystem.LAUNCHER_2_MIN_VELOCITY

        // 1. Spin up launcher flywheels
        robot.launcher?.velocity = targetVel1
        robot.launcher2?.velocity = targetVel2

        val startTime = System.currentTimeMillis()
        var flywheelsReady = false

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < launchDurationMs) {
            val v1 = robot.launcher?.velocity ?: 0.0
            val v2 = robot.launcher2?.velocity ?: 0.0

            // Once flywheels reach correct velocity/position threshold, trigger left servo and left intake servo
            if (!flywheelsReady && ((v1 > minVel1 && v2 > minVel2) || (System.currentTimeMillis() - startTime) > 800)) {
                flywheelsReady = true
            }

            if (flywheelsReady) {
                robot.leftServo?.power = 1.0
                robot.leftIntakeServo?.power = 0.8
                robot.windmillServo?.power = 1.0
                robot.intake?.power = 0.8
            } else {
                robot.leftServo?.power = 0.0
                robot.leftIntakeServo?.power = 0.0
            }

            updatePoseAndTelemetry()
            sleep(20)
        }

        // 2. Stop launcher, windmill, intake, and servos
        robot.launcher?.velocity = 0.0
        robot.launcher2?.velocity = 0.0
        robot.windmillServo?.power = 0.0
        robot.intake?.power = 0.0
        robot.leftIntakeServo?.power = 0.0
        robot.rightIntakeServo?.power = 0.0
        robot.leftServo?.power = 0.0
        robot.rightServo?.power = 0.0
    }

    private fun runBlockingAction(action: Action) {
        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        action.preview(c)

        var running = true
        while (running && opModeIsActive()) {
            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(c.operations)
            running = action.run(packet)
            dash.sendTelemetryPacket(packet)

            val currentPose = drive.updatePose()
            updateAutoTelemetry(currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()))
        }
        drive.stop()
    }

    private fun updatePoseAndTelemetry() {
        val currentPose = drive.updatePose()
        updateAutoTelemetry(currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()))
    }

    /**
     * Data class holding detected AprilTag information from Limelight.
     */
    data class AprilTagTarget(
        val id: Int,
        val txDegrees: Double,
        val distanceInches: Double,
        val isValid: Boolean
    )

    /**
     * Checks if the Limelight camera is live and querying valid AprilTag detections.
     * Returns tag target information if found, or null otherwise.
     */
    private fun getAprilTagTarget(): AprilTagTarget? {
        val limelight = robot.limelight ?: return null
        val status = limelight.status ?: return null

        // Check if camera is live / connected
        if (status.fps <= 0.0) return null

        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null

        // Check fiducial results (AprilTags)
        val fiducials = result.fiducialResults
        if (fiducials.isNullOrEmpty()) {
            // Fallback: If pipeline has a valid target lock (tx/ty)
            if (result.tx != 0.0 || result.ty != 0.0) {
                val botpose = result.botpose
                val distInches = if (botpose != null) {
                    hypot(botpose.position.x, botpose.position.y) * METERS_TO_INCHES
                } else {
                    0.0
                }
                return AprilTagTarget(
                    id = -1,
                    txDegrees = result.tx,
                    distanceInches = distInches,
                    isValid = true
                )
            }
            return null
        }

        // Pick the closest or first detected AprilTag
        var bestTag: AprilTagTarget? = null
        var minDistance = Double.MAX_VALUE

        for (fr in fiducials) {
            val pose = fr.targetPoseCameraSpace
            val distInches = if (pose != null) {
                val x = pose.position.x
                val y = pose.position.y
                val z = pose.position.z
                sqrt(x * x + y * y + z * z) * METERS_TO_INCHES
            } else {
                0.0
            }

            if (distInches > 0.0 && distInches < minDistance) {
                minDistance = distInches
                bestTag = AprilTagTarget(
                    id = fr.fiducialId,
                    txDegrees = fr.targetXDegrees,
                    distanceInches = distInches,
                    isValid = true
                )
            } else if (bestTag == null) {
                bestTag = AprilTagTarget(
                    id = fr.fiducialId,
                    txDegrees = fr.targetXDegrees,
                    distanceInches = distInches,
                    isValid = true
                )
            }
        }

        return bestTag
    }

    /**
     * Leverages the live Limelight AprilTag detection to align and approach the tag
     * until the robot reaches [targetDistanceInches].
     */
    private fun approachAprilTagIfDetected(targetDistanceInches: Double, timeoutSec: Double = 3.0) {
        val startTime = System.currentTimeMillis()
        var tagSeen = false

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < (timeoutSec * 1000)) {
            val tag = getAprilTagTarget()
            val currentPose = drive.updatePose()

            if (tag != null && tag.isValid) {
                tagSeen = true
                val bearingErrorDeg = tag.txDegrees
                val rangeErrorInches = if (tag.distanceInches > 0.0) {
                    tag.distanceInches - targetDistanceInches
                } else {
                    0.0
                }

                updateAutoTelemetry(
                    currentPose.position.x,
                    currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble()),
                    tag,
                    targetDistanceInches
                )

                // Stop if we are within 1.5 inches of target distance and within 2 degrees of alignment
                if (abs(bearingErrorDeg) < 2.0 && (tag.distanceInches == 0.0 || abs(rangeErrorInches) < 1.5)) {
                    currentStep = "Limelight Tag Alignment Achieved"
                    updateAutoTelemetry(
                        currentPose.position.x,
                        currentPose.position.y,
                        Math.toDegrees(currentPose.heading.toDouble()),
                        tag,
                        targetDistanceInches
                    )
                    break
                }

                val forwardPower = if (tag.distanceInches > 0.0) {
                    (rangeErrorInches * SPEED_P).coerceIn(-MAX_APPROACH_SPEED, MAX_APPROACH_SPEED)
                } else {
                    0.0
                }

                // Bearing error: positive tx means tag is to right, so robot needs to turn clockwise (right)
                // Tank drive: left positive, right negative turns clockwise
                val turnPower = (bearingErrorDeg * TURN_P).coerceIn(-MAX_APPROACH_TURN, MAX_APPROACH_TURN)

                val leftPower = (forwardPower + turnPower).coerceIn(-1.0, 1.0)
                val rightPower = (forwardPower - turnPower).coerceIn(-1.0, 1.0)

                drive.setDrivePowers(leftPower, rightPower)
            } else {
                drive.stop()
                currentStep = if (tagSeen) "Lost Tag View - Holding" else "Searching for Tag..."
                updateAutoTelemetry(
                    currentPose.position.x,
                    currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble())
                )
            }

            sleep(20)
        }

        drive.stop()
        telemetry.update()
    }

    /**
     * Turns in place to an absolute target field heading (in degrees) using odometry IMU heading.
     */
    private fun turnToHeading(targetAngleDegrees: Double, speed: Double, timeoutSec: Double = 4.0) {
        val startTime = System.currentTimeMillis()

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < (timeoutSec * 1000)) {
            val pose = drive.updatePose()
            val currentHeadingDeg = Math.toDegrees(pose.heading.toDouble())

            updateAutoTelemetry(pose.position.x, pose.position.y, currentHeadingDeg)

            var angleError = targetAngleDegrees - currentHeadingDeg
            while (angleError > 180.0) angleError -= 360.0
            while (angleError < -180.0) angleError += 360.0

            if (abs(angleError) < 2.0) break

            val turnPower = (angleError * 0.015).coerceIn(-speed, speed)
            drive.setDrivePowers(-turnPower, turnPower)
            sleep(20)
        }

        drive.stop()
    }

    /**
     * Drives to an absolute field coordinate (targetX, targetY) using odometry feedback.
     * First rotates to face the target point, then drives towards it with heading correction.
     */
    private fun driveToPoint(targetX: Double, targetY: Double, speed: Double, timeoutSec: Double = 6.0) {
        val startTime = System.currentTimeMillis()

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < (timeoutSec * 1000)) {
            val pose = drive.updatePose()
            val dx = targetX - pose.position.x
            val dy = targetY - pose.position.y
            val distanceRemaining = hypot(dx, dy)

            updateAutoTelemetry(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()))

            // Target reached within tolerance
            if (distanceRemaining < 1.5) break

            // Desired heading to face the target point
            val desiredHeadingRad = atan2(dy, dx)
            var headingError = desiredHeadingRad - pose.heading.toDouble()
            while (headingError > Math.PI) headingError -= 2 * Math.PI
            while (headingError < -Math.PI) headingError += 2 * Math.PI

            // If heading error is large, pivot in place first
            if (abs(Math.toDegrees(headingError)) > 30.0) {
                val turnPower = (headingError * 0.6).coerceIn(-0.4, 0.4)
                drive.setDrivePowers(-turnPower, turnPower)
            } else {
                val headingCorrection = (headingError * 0.8).coerceIn(-0.3, 0.3)
                val forwardPower = (distanceRemaining * 0.05).coerceIn(0.15, speed)
                val leftPower = (forwardPower - headingCorrection).coerceIn(-1.0, 1.0)
                val rightPower = (forwardPower + headingCorrection).coerceIn(-1.0, 1.0)
                drive.setDrivePowers(leftPower, rightPower)
            }

            sleep(20)
        }

        drive.stop()
    }

    /**
     * Comprehensive telemetry and Dashboard streaming:
     * - Odometry (X, Y, Heading)
     * - Drivetrain motor powers
     * - Launcher and Intake states
     * - Limelight AprilTag targeting status and 3D pose
     * - Field overlay drawing
     */
    private fun updateAutoTelemetry(
        x: Double,
        y: Double,
        headingDeg: Double,
        tag: AprilTagTarget? = null,
        targetDistanceInches: Double = DESIRED_TAG_DISTANCE_INCHES
    ) {
        val launcherVel = robot.launcher?.velocity ?: 0.0
        val launcher2Vel = robot.launcher2?.velocity ?: 0.0
        val leftPower = robot.leftDrive?.power ?: 0.0
        val rightPower = robot.rightDrive?.power ?: 0.0
        val intakePower = robot.intake?.power ?: 0.0
        val limelightStatus = robot.limelight?.status

        // --- FTC Dashboard Packet ---
        val packet = TelemetryPacket()
        packet.put("Auto Step", currentStep)
        packet.put("Auto X (in)", x)
        packet.put("Auto Y (in)", y)
        packet.put("Auto Heading (deg)", headingDeg)
        packet.put("Left Drive Power", leftPower)
        packet.put("Right Drive Power", rightPower)
        packet.put("Launcher 1 Velocity", launcherVel)
        packet.put("Launcher 2 Velocity", launcher2Vel)
        packet.put("Intake Power", intakePower)

        // Limelight telemetry in packet
        if (limelightStatus != null) {
            packet.put("Limelight FPS", limelightStatus.fps)
            packet.put("Limelight Temp (°C)", limelightStatus.temp)
        }
        if (tag != null && tag.isValid) {
            packet.put("Target Tag ID", tag.id)
            packet.put("Tag Bearing Error (°)", tag.txDegrees)
            packet.put("Tag Distance (in)", tag.distanceInches)
            packet.put("Tag Range Error (in)", tag.distanceInches - targetDistanceInches)
        }

        // Field View Overlay
        val fieldOverlay = packet.fieldOverlay()
        fieldOverlay.setStroke("#4CAF50")
        fieldOverlay.strokeCircle(x, y, 9.0)

        val headingRad = Math.toRadians(headingDeg)
        val length = 12.0
        fieldOverlay.strokeLine(x, y, x + length * cos(headingRad), y + length * sin(headingRad))

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        // --- Driver Station Telemetry ---
        telemetry.addData("--- Autonomous Status ---", "")
        telemetry.addData("Step", currentStep)
        telemetry.addData("Odometry Pose", "X: %.2f in, Y: %.2f in, Head: %.1f°", x, y, headingDeg)
        telemetry.addData("Drive Motors", "Left: %.2f, Right: %.2f", leftPower, rightPower)
        telemetry.addData("Launcher / Intake", "Vel: %.1f ticks/sec, Intake: %.2f", launcherVel, intakePower)

        telemetry.addData("--- Limelight & Vision ---", "")
        if (limelightStatus != null && limelightStatus.fps > 0.0) {
            telemetry.addData("Limelight", "Online (%.0f FPS, %.1f°C)", limelightStatus.fps, limelightStatus.temp)
            if (tag != null && tag.isValid) {
                val rangeErr = if (tag.distanceInches > 0.0) tag.distanceInches - targetDistanceInches else 0.0
                telemetry.addData("AprilTag #%d", tag.id)
                telemetry.addData("Tag Bearing Err", "%.2f°", tag.txDegrees)
                telemetry.addData("Tag Distance", "%.1f in (err: %.1f in)", tag.distanceInches, rangeErr)
            } else {
                telemetry.addData("AprilTag", "No target detected")
            }
        } else {
            telemetry.addData("Limelight", "Offline or Not Streaming")
        }

        telemetry.update()
    }
}
