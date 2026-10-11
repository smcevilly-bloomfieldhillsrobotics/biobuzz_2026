package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Autonomous OpMode for BioBuzz StarterBot (Tank Drive) using goBILDA Pinpoint Odometry
 * and Limelight 3A AprilTag targeting.
 *
 * Drives to field waypoints, checks if the Limelight camera is live and an AprilTag is detected,
 * and if detected, smoothly aligns and approaches the AprilTag to a calibrated target distance.
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

    override fun runOpMode() {
        robot.init(hardwareMap)
        drive = RoadRunnerTankDrive(robot)

        // Reset starting odometry pose to field origin (0, 0, 0 rad)
        robot.pinpoint?.resetPosAndIMU()

        telemetry.addData("Status", "Autonomous Ready")
        telemetry.update()

        waitForStart()

        if (!opModeIsActive()) return

        // Step 1: Drive forward 30 inches on the field
        telemetry.addData("Auto Step", "1: Driving forward 30 inches")
        telemetry.update()
        publishDashboard(56.0, 8.0, 0.0)
        driveStraightInches(targetDistanceInches = 30.0, speed = 0.5)

        sleep(500)
        drive.updatePose().let { publishDashboard(it.position.x, it.position.y, Math.toDegrees(it.heading.toDouble())) }

        // Step 2: Turn 90 degrees counter-clockwise (positive heading)
        telemetry.addData("Auto Step", "2: Turning 90 degrees")
        telemetry.update()
        turnDegrees(targetAngleDegrees = 90.0, speed = 0.4)

        sleep(500)
        drive.updatePose().let { publishDashboard(it.position.x, it.position.y, Math.toDegrees(it.heading.toDouble())) }

        // Step 3: Drive forward another 20 inches to target waypoint
        telemetry.addData("Auto Step", "3: Driving forward 20 inches to target area")
        telemetry.update()
        driveStraightInches(targetDistanceInches = 20.0, speed = 0.5)

        // Step 4: Check if Limelight is live and leverage AprilTag to approach
        telemetry.addData("Auto Step", "4: Checking Limelight AprilTag...")
        telemetry.update()
        approachAprilTagIfDetected(targetDistanceInches = DESIRED_TAG_DISTANCE_INCHES, timeoutSec = 5.0)

        // Step 5: Navigate back to (0, 0)
        telemetry.addData("Auto Step", "5: Driving to (0, 0)")
        telemetry.update()
        driveToPoint(targetX = 0.0, targetY = 0.0, speed = 0.5)

        telemetry.addData("Auto Step", "Complete! Holding position at (0, 0).")
        telemetry.update()

        // Keep updating telemetry & dashboard while active
        while (opModeIsActive()) {
            val currentPose = drive.updatePose()
            publishDashboard(currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()))
            sleep(20)
        }

        drive.stop()
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
    private fun approachAprilTagIfDetected(targetDistanceInches: Double, timeoutSec: Double = 5.0) {
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

                telemetry.addData("Limelight", "Target Tag #${tag.id} Locked")
                telemetry.addData("Tag Bearing Error", "%.2f°", bearingErrorDeg)
                telemetry.addData("Tag Dist", "%.1f in (err: %.1f)", tag.distanceInches, rangeErrorInches)
                publishDashboard(currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()))

                // Stop if we are within 1.5 inches of target distance and within 2 degrees of alignment
                if (abs(bearingErrorDeg) < 2.0 && (tag.distanceInches == 0.0 || abs(rangeErrorInches) < 1.5)) {
                    telemetry.addData("Limelight", "Target distance & alignment reached!")
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
                if (tagSeen) {
                    telemetry.addData("Limelight", "Lost tag view during approach")
                } else {
                    telemetry.addData("Limelight", "No AprilTag visible from waypoint; holding position")
                }
                publishDashboard(currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()))
            }

            sleep(20)
        }

        drive.stop()
        telemetry.update()
    }

    /**
     * Drives straight using closed-loop odometry feedback.
     */
    private fun driveStraightInches(targetDistanceInches: Double, speed: Double, timeoutSec: Double = 5.0) {
        val startPose = drive.updatePose()
        val startX = startPose.position.x
        val startY = startPose.position.y
        val targetHeading = startPose.heading.toDouble()
        val startTime = System.currentTimeMillis()

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < (timeoutSec * 1000)) {
            val pose = drive.updatePose()
            val distanceTraveled = hypot(pose.position.x - startX, pose.position.y - startY)

            publishDashboard(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()))

            if (distanceTraveled >= targetDistanceInches) break

            // Simple P-controller on heading to keep driving in a straight line
            var headingError = targetHeading - pose.heading.toDouble()
            while (headingError > Math.PI) headingError -= 2 * Math.PI
            while (headingError < -Math.PI) headingError += 2 * Math.PI

            val headingCorrection = (headingError * 0.8).coerceIn(-0.3, 0.3)
            val leftPower = (speed - headingCorrection).coerceIn(-1.0, 1.0)
            val rightPower = (speed + headingCorrection).coerceIn(-1.0, 1.0)

            drive.setDrivePowers(leftPower, rightPower)
        }

        drive.stop()
    }

    /**
     * Turns in place to a relative target angle using the odometry IMU heading.
     */
    private fun turnDegrees(targetAngleDegrees: Double, speed: Double, timeoutSec: Double = 4.0) {
        val startPose = drive.updatePose()
        val startHeadingDeg = Math.toDegrees(startPose.heading.toDouble())
        val targetHeadingDeg = startHeadingDeg + targetAngleDegrees
        val startTime = System.currentTimeMillis()

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < (timeoutSec * 1000)) {
            val pose = drive.updatePose()
            val currentHeadingDeg = Math.toDegrees(pose.heading.toDouble())

            publishDashboard(pose.position.x, pose.position.y, currentHeadingDeg)

            var angleError = targetHeadingDeg - currentHeadingDeg
            while (angleError > 180.0) angleError -= 360.0
            while (angleError < -180.0) angleError += 360.0

            if (abs(angleError) < 2.0) break

            val turnPower = (angleError * 0.015).coerceIn(-speed, speed)
            drive.setDrivePowers(-turnPower, turnPower)
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

            publishDashboard(pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()))

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

    private fun publishDashboard(x: Double, y: Double, headingDeg: Double) {
        val launcherVel = robot.launcher?.velocity ?: 0.0

        val packet = TelemetryPacket()
        packet.put("Auto X (in)", x)
        packet.put("Auto Y (in)", y)
        packet.put("Auto Heading (deg)", headingDeg)
        packet.put("Launcher Velocity", launcherVel)

        val fieldOverlay = packet.fieldOverlay()
        fieldOverlay.setStroke("#4CAF50")
        fieldOverlay.strokeCircle(x, y, 9.0)

        val headingRad = Math.toRadians(headingDeg)
        val length = 12.0
        fieldOverlay.strokeLine(x, y, x + length * cos(headingRad), y + length * sin(headingRad))

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        telemetry.addData("Pose", "X: %.1f in, Y: %.1f in, Heading: %.1f°", x, y, headingDeg)
        telemetry.addData("Launcher Vel", "%.1f ticks/sec", launcherVel)
        telemetry.update()
    }
}
