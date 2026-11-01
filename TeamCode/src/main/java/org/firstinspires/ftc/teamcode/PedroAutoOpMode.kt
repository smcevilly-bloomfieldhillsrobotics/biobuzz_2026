package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.paths.Path
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Autonomous OpMode for BioBuzz StarterBot using Pedro Pathing 3.0 & Pinpoint Odometry.
 * Consistent with RoadRunnerAutoOpMode:
 * - Executes launcher spin-up & 0.2s feeder pulse BEFORE path traversal
 * - Traverses all 4 field poses defined in PedroPaths (shooting, reloading, garden, opposite shooting)
 * - Uses Limelight 3A AprilTag targeting & approach at the shooting station
 * - Features closed-loop odometry & Pedro DrivePowers navigation
 */
@Autonomous(name = "BioBuzz Autonomous (Pedro Pathing)", group = "Autonomous")
class PedroAutoOpMode : LinearOpMode() {

    companion object {
        const val DESIRED_TAG_DISTANCE_INCHES = 24.0
        const val METERS_TO_INCHES = 39.3701
        const val SPEED_P = 0.025
        const val TURN_P = 0.015
        const val MAX_APPROACH_SPEED = 0.40
        const val MAX_APPROACH_TURN = 0.30
    }

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var odometry: OdometrySubsystem
    private val paths = PedroPaths()
    private var currentStep = "Initializing"

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)
        robot.initLimelight(hardwareMap)
        robot.initPinpoint(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)
        odometry = OdometrySubsystem(robot)

        // Initialize odometry to default startPose (56.0, 8.0, 90°)
        odometry.resetPosition(
            paths.startPose.x(),
            paths.startPose.y(),
            Math.toDegrees(paths.startPose.heading())
        )

        // Continuous init inspection loop until START button is pressed
        currentStep = "Init - Hardware Verification & Pose Inspection"

        while (!isStarted && !isStopRequested) {
            odometry.update(null)

            val pinpointOk = robot.pinpoint != null
            val limelightStatus = robot.limelight?.status
            val limelightOk = (limelightStatus?.fps ?: 0.0) > 0.0
            val motorsOk = robot.frontLeftDrive != null && robot.frontRightDrive != null &&
                           robot.backLeftDrive != null && robot.backRightDrive != null
            val launchersOk = robot.launcher != null && robot.launcher2 != null

            val currentTag = getAprilTagTarget()

            updateAutoTelemetry(
                x = odometry.posX,
                y = odometry.posY,
                headingDeg = odometry.heading,
                tag = currentTag,
                pinpointOk = pinpointOk,
                limelightOk = limelightOk,
                motorsOk = motorsOk,
                launchersOk = launchersOk
            )

            sleep(30)
        }

        if (!opModeIsActive()) return

        // Step 1: Follow Path to Shooting Position (56.0, 36.0, 90°)
        currentStep = "1: Following Path -> Shooting Position (56.0, 36.0)"
        followPedroPath(paths.shootingPositionPath())
        sleep(250)

        // Step 1b: Limelight AprilTag Align & Approach
        currentStep = "Limelight AprilTag Align & Approach"
        val tagDetected = approachAprilTagIfDetected(targetDistanceInches = DESIRED_TAG_DISTANCE_INCHES, timeoutSec = 3.0)
        sleep(200)

        // Step 1c: Execute Launcher Function ONLY IF AprilTag is detected
        if (tagDetected) {
            currentStep = "Launching game elements at Shooting Position (AprilTag Confirmed)"
            updatePoseAndTelemetry()
            executeLaunchSequence(launchDurationMs = 2500, requireAprilTag = true)
            sleep(200)
        } else {
            currentStep = "Skipping Shooting: No AprilTag Detected in Field View"
            updatePoseAndTelemetry()
            sleep(1000)
        }

        // Step 2: Follow Path to Reloading Start (34.80, 10.97, 180°)
        currentStep = "2: Following Path -> Reloading Start (34.80, 10.97)"
        followPedroPath(paths.reloadingStartPath())
        sleep(250)

        // Step 3: Follow Path to Load Garden (0.83, 14.19, -5.42°)
        currentStep = "3: Following Path -> Load Garden (0.83, 14.19)"
        followPedroPath(paths.loadGardenPath())
        sleep(250)

        // Step 4: Follow Path to Opposite Shooting Position (52.82, 126.89, 270°)
        currentStep = "4: Following Path -> Opposite Shooting (52.82, 126.89)"
        followPedroPath(paths.oppositeShootingPositionPath())
        sleep(250)

        currentStep = "Complete! Holding final position"

        while (opModeIsActive()) {
            pedroMecanum.stop()
            updatePoseAndTelemetry()
            sleep(20)
        }

        pedroMecanum.stop()
    }

    /**
     * Follows a Pedro Path smoothly using Pedro DrivePowers kinematics and odometry feedback.
     */
    private fun followPedroPath(path: Path, timeoutSec: Double = 6.0) {
        val targetPose = path.endPose()
        val startTime = System.currentTimeMillis()

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < (timeoutSec * 1000)) {
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

            updatePoseAndTelemetry()

            if (distanceRemaining < 1.5 && abs(Math.toDegrees(headingError)) < 3.0) {
                break
            }

            // Proportional feedback in field frame (X and Y displacement)
            val fieldXCmd = (dx * 0.06).coerceIn(-0.6, 0.6)
            val fieldYCmd = (dy * 0.06).coerceIn(-0.6, 0.6)
            val turnPower = (headingError * 0.7).coerceIn(-0.4, 0.4)

            // Convert field-frame vector to robot-frame vector given current robot heading
            val cosH = cos(currentHeadingRad)
            val sinH = sin(currentHeadingRad)

            val robotForward = fieldXCmd * cosH + fieldYCmd * sinH
            val robotStrafe = -fieldXCmd * sinH + fieldYCmd * cosH

            pedroMecanum.drive(DrivePowers(robotForward, robotStrafe, turnPower), true)
            sleep(15)
        }

        pedroMecanum.stop()
    }

    /**
     * Spins up the launcher flywheels and feeds elements via left servos once target speed is reached.
     * Enforces AprilTag visibility check so launcher will NOT fire if no AprilTag is detected.
     */
    private fun executeLaunchSequence(launchDurationMs: Long = 2500, requireAprilTag: Boolean = true) {
        if (requireAprilTag) {
            val tag = getAprilTagTarget()
            if (tag == null || !tag.isValid) {
                currentStep = "Launch Aborted: No AprilTag Detected!"
                updatePoseAndTelemetry()
                sleep(500)
                return
            }
        }

        val targetVel1 = IntakeLauncherSubsystem.TARGET_VELOCITY
        val minVel1 = IntakeLauncherSubsystem.MIN_VELOCITY
        val targetVel2 = IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY
        val minVel2 = IntakeLauncherSubsystem.LAUNCHER_2_MIN_VELOCITY

        robot.launcher?.velocity = targetVel1
        robot.launcher2?.velocity = targetVel2

        val startTime = System.currentTimeMillis()
        var flywheelsReady = false

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < launchDurationMs) {
            val v1 = robot.launcher?.velocity ?: 0.0
            val v2 = robot.launcher2?.velocity ?: 0.0

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

        robot.launcher?.velocity = 0.0
        robot.launcher2?.velocity = 0.0
        robot.windmillServo?.power = 0.0
        robot.intake?.power = 0.0
        robot.leftIntakeServo?.power = 0.0
        robot.leftServo?.power = 0.0
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
                val distInches = if (botpose != null) hypot(botpose.position.x, botpose.position.y) * METERS_TO_INCHES else 0.0
                return AprilTagTarget(id = -1, txDegrees = result.tx, distanceInches = distInches, isValid = true)
            }
            return null
        }

        var bestTag: AprilTagTarget? = null
        var minDistance = Double.MAX_VALUE

        for (fr in fiducials) {
            val pose = fr.targetPoseCameraSpace
            val distInches = if (pose != null) sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z) * METERS_TO_INCHES else 0.0

            if (distInches > 0.0 && distInches < minDistance) {
                minDistance = distInches
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            } else if (bestTag == null) {
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            }
        }

        return bestTag
    }

    private fun approachAprilTagIfDetected(targetDistanceInches: Double, timeoutSec: Double = 3.0): Boolean {
        val startTime = System.currentTimeMillis()
        var tagSeen = false

        while (opModeIsActive() && ((System.currentTimeMillis() - startTime) < (timeoutSec * 1000))) {
            val tag = getAprilTagTarget()
            odometry.update(null)

            if (tag != null && tag.isValid) {
                tagSeen = true
                val bearingErrorDeg = tag.txDegrees
                val rangeErrorInches = if (tag.distanceInches > 0.0) tag.distanceInches - targetDistanceInches else 0.0

                updateAutoTelemetry(odometry.posX, odometry.posY, odometry.heading, tag, targetDistanceInches)

                if (abs(bearingErrorDeg) < 2.0 && (tag.distanceInches == 0.0 || abs(rangeErrorInches) < 1.5)) {
                    currentStep = "Limelight Tag Alignment Achieved"
                    break
                }

                val forwardPower = if (tag.distanceInches > 0.0) (rangeErrorInches * SPEED_P).coerceIn(-MAX_APPROACH_SPEED, MAX_APPROACH_SPEED) else 0.0
                val turnPower = (bearingErrorDeg * TURN_P).coerceIn(-MAX_APPROACH_TURN, MAX_APPROACH_TURN)

                pedroMecanum.drive(DrivePowers(forwardPower, 0.0, turnPower), true)
            } else {
                pedroMecanum.stop()
                currentStep = if (tagSeen) "Lost Tag View - Holding" else "Searching for Tag..."
                updateAutoTelemetry(odometry.posX, odometry.posY, odometry.heading)
            }

            sleep(20)
        }

        pedroMecanum.stop()
        return tagSeen
    }

    private fun updatePoseAndTelemetry() {
        odometry.update(null)
        updateAutoTelemetry(odometry.posX, odometry.posY, odometry.heading)
    }

    private fun updateAutoTelemetry(
        x: Double,
        y: Double,
        headingDeg: Double,
        tag: AprilTagTarget? = null,
        targetDistanceInches: Double = DESIRED_TAG_DISTANCE_INCHES,
        pinpointOk: Boolean = robot.pinpoint != null,
        limelightOk: Boolean = (robot.limelight?.status?.fps ?: 0.0) > 0.0,
        motorsOk: Boolean = robot.frontLeftDrive != null && robot.frontRightDrive != null && robot.backLeftDrive != null && robot.backRightDrive != null,
        launchersOk: Boolean = robot.launcher != null && robot.launcher2 != null
    ) {
        val launcherVel = robot.launcher?.velocity ?: 0.0
        val launcher2Vel = robot.launcher2?.velocity ?: 0.0
        val limelightStatus = robot.limelight?.status

        val wheelPowers = pedroMecanum.wheelPowers
        val flPower = if (wheelPowers.isNotEmpty()) wheelPowers[0] else 0.0
        val frPower = if (wheelPowers.size > 1) wheelPowers[1] else 0.0
        val blPower = if (wheelPowers.size > 2) wheelPowers[2] else 0.0
        val brPower = if (wheelPowers.size > 3) wheelPowers[3] else 0.0

        val packet = TelemetryPacket()
        packet.put("Auto Step", currentStep)
        packet.put("Auto X (in)", x)
        packet.put("Auto Y (in)", y)
        packet.put("Auto Heading (deg)", headingDeg)
        packet.put("FL Power", flPower)
        packet.put("FR Power", frPower)
        packet.put("BL Power", blPower)
        packet.put("BR Power", brPower)
        packet.put("Launcher 1 Vel", launcherVel)
        packet.put("Launcher 2 Vel", launcher2Vel)

        packet.put("HW: Pinpoint Odometry", if (pinpointOk) "OK" else "FAILED")
        packet.put("HW: Limelight Camera", if (limelightOk) "OK" else "OFFLINE")
        packet.put("HW: Drivetrain Motors", if (motorsOk) "OK" else "INCOMPLETE")
        packet.put("HW: Launcher Flywheels", if (launchersOk) "OK" else "INCOMPLETE")

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

        // Draw BioBuzz Field & Robot Overlay on FTC Dashboard
        drawBioBuzzFieldOverlay(packet.fieldOverlay(), x, y, headingDeg)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        telemetry.addData("--- Pedro Autonomous Status ---", "")
        telemetry.addData("Step", currentStep)
        telemetry.addData("Pose (Live Inspection)", "X: %.2f in, Y: %.2f in, Heading: %.1f°", x, y, headingDeg)

        telemetry.addData("--- Hardware Inspection ---", "")
        telemetry.addData("Pinpoint Odometry", if (pinpointOk) "OPERATIONAL" else "NOT CONNECTED")
        telemetry.addData("Limelight Camera", if (limelightOk) "ONLINE (%.0f FPS)" else "OFFLINE", limelightStatus?.fps ?: 0.0)
        telemetry.addData("Mecanum Drivetrain", if (motorsOk) "OPERATIONAL (4 Motors)" else "MOTOR MISSING")
        telemetry.addData("Launcher Flywheels", if (launchersOk) "OPERATIONAL (Dual Motors)" else "MOTOR MISSING")

        telemetry.addData("--- Subsystem Metrics ---", "")
        telemetry.addData("Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", flPower, frPower, blPower, brPower)
        telemetry.addData("Launchers", "L1: %.0f, L2: %.0f", launcherVel, launcher2Vel)

        val activeTag = tag ?: getAprilTagTarget()
        if (activeTag != null && activeTag.isValid) {
            val tagIdStr = if (activeTag.id >= 0) "#${activeTag.id}" else "Lock"
            telemetry.addData("Limelight AprilTag", "%s | Dist: %.1f in | Offset: %.1f°",
                tagIdStr, activeTag.distanceInches, activeTag.txDegrees)
        } else {
            telemetry.addData("Limelight AprilTag", "No AprilTag Target Detected")
        }
        telemetry.update()
    }

    private fun drawBioBuzzFieldOverlay(fieldOverlay: Canvas, robotX: Double, robotY: Double, headingDeg: Double) {
        // Field Perimeter (144" x 144")
        fieldOverlay.setStroke("#52525B")
        fieldOverlay.strokeRect(0.0, 0.0, 144.0, 144.0)

        // Red HIVE
        fieldOverlay.setStroke("#DC2626")
        fieldOverlay.strokeRect(46.0, 50.0, 2.6, 41.4)

        // Blue HIVE
        fieldOverlay.setStroke("#2563EB")
        fieldOverlay.strokeRect(93.9, 50.0, 2.6, 41.4)

        // FLOWER Stations
        fieldOverlay.setStroke("#EF4444")
        fieldOverlay.strokeCircle(46.75, 139.5, 4.0) // Top Scoring FLOWER
        fieldOverlay.strokeCircle(2.0, 47.0, 4.0)   // Red Alliance FLOWER

        fieldOverlay.setStroke("#3B82F6")
        fieldOverlay.strokeCircle(94.5, 2.0, 4.0)   // Audience FLOWER
        fieldOverlay.strokeCircle(139.5, 94.75, 4.0)// Blue Alliance FLOWER

        // Robot Body (18" x 18" Circle & Heading Line)
        fieldOverlay.setStroke("#22C55E")
        fieldOverlay.strokeCircle(robotX, robotY, 9.0)

        val headingRad = Math.toRadians(headingDeg)
        val length = 12.0
        fieldOverlay.strokeLine(
            robotX,
            robotY,
            robotX + length * cos(headingRad),
            robotY + length * sin(headingRad)
        )
    }
}
