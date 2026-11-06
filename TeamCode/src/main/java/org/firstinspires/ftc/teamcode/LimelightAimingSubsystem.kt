/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import kotlin.math.hypot
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

@Config
class LimelightAimingSubsystem(hardwareMap: HardwareMap) {

    var limelight: Limelight3A? = null
    var cameraMotor: DcMotor? = null

    companion object {
        @JvmField var Kp = 0.008 // Proportional gain for direct angle-to-power mapping
        @JvmField var MAX_POWER = 0.20 // Maximum motor power cap for smooth control
        @JvmField var MAX_ACCEL = 0.75 // Slew-rate limit for smooth acceleration ramping
        @JvmField var PIVOT_POWER = 0.50 // Fast pivot power when tag is inverted
        @JvmField var DEADBAND_DEGREES = 0.5 // Deadband threshold where motor stops completely
        @JvmField var ADJUSTMENT_INTERVAL = 0.2 // Time interval (seconds) between position recalculations
        @JvmField var FILTER_ALPHA = 0.6 // Low-pass filter factor for smooth motion over time
        @JvmField var INVERT_MOTOR = false

        // --- PRECISE PHYSICAL OFFSETS FOR HIVE CELL TAG GROUPS ---
        @JvmField var INNER_TAG_OFFSET_METERS = 0.047625 // 1.875 inches from centerline to inner tag center
        @JvmField var OUTER_TAG_OFFSET_METERS = 0.13017  // 5.125 inches from centerline to outer tag center
    }

    var isInitialized = false
        private set
    var matchedGroupName = "None"
        private set
    var lastErrorTx = 0.0
        private set
    var lastErrorTy = 0.0
        private set
    var lastDistance3D = 0.0
        private set
    var lastMotorPower = 0.0
        private set
    var isTagInverted = false
        private set
    val lastDetectedTags = mutableListOf<Int>()

    private val pidTimer = ElapsedTime()
    private val adjustmentTimer = ElapsedTime()
    private var lastError = 0.0

    init {
        try {
            limelight = hardwareMap.get(Limelight3A::class.java, "limelight")?.apply {
                setPollRateHz(100)
                start()
                pipelineSwitch(0)
            }
            cameraMotor = hardwareMap.get(DcMotor::class.java, "camera_motor")?.apply {
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
            isInitialized = limelight?.status != null
            pidTimer.reset()
            adjustmentTimer.reset()
        } catch (_: Exception) {
            isInitialized = false
        }
    }

    fun update(packet: TelemetryPacket?) {
        lastDetectedTags.clear()
        isTagInverted = false
        lastDistance3D = 0.0
        val motor = cameraMotor ?: return

        val status = limelight?.status ?: run {
            isInitialized = false
            motor.power = 0.0
            return
        }

        val result = limelight?.latestResult
        if (result == null || !result.isValid) {
            lastErrorTx = 0.0
            lastErrorTy = 0.0
            motor.power = 0.0
            lastMotorPower = 0.0
            adjustmentTimer.reset()
            return
        }

        val totalLatency = result.captureLatency + result.targetingLatency

        val tagPoseMap = mutableMapOf<Int, Pose3D>()
        val tagTyMap = mutableMapOf<Int, Double>()
        var primaryDistance3D = 0.0

        result.fiducialResults?.forEach { fr ->
            val id = fr.fiducialId
            if (id in 30..45) {
                lastDetectedTags.add(id)
                tagTyMap[id] = fr.targetYDegrees
                fr.targetPoseCameraSpace?.let { pose ->
                    tagPoseMap[id] = pose
                    val x = pose.position.x
                    val y = pose.position.y
                    val z = pose.position.z
                    val d = sqrt(x * x + y * y + z * z)
                    if (primaryDistance3D == 0.0) {
                        primaryDistance3D = d
                    }

                    val roll = pose.orientation.getRoll(AngleUnit.DEGREES)
                    val pitch = pose.orientation.getPitch(AngleUnit.DEGREES)
                    if (abs(roll) > 90.0 || abs(pitch) > 90.0) {
                        isTagInverted = true
                    }
                }
            }
        }

        lastDistance3D = primaryDistance3D

        var bestX = 0.0
        var bestZ = 0.0
        var bestTy = 0.0
        matchedGroupName = "None"
        var targetFound = false

        // Check the 4 groups of tags:
        // Group 0: 30, 31, 32, 33 (Center between 31 & 32)
        // Group 1: 34, 35, 36, 37 (Center between 35 & 36)
        // Group 2: 38, 39, 40, 41 (Center between 39 & 40)
        // Group 3: 42, 43, 44, 45 (Center between 43 & 44)
        for (groupIndex in 0..3) {
            val baseId = 30 + (groupIndex * 4)
            val outerLeft = baseId
            val innerLeft = baseId + 1  // e.g. 31, 35, 39, 43
            val innerRight = baseId + 2 // e.g. 32, 36, 40, 44
            val outerRight = baseId + 3

            val pOL = tagPoseMap[outerLeft]
            val pIL = tagPoseMap[innerLeft]
            val pIR = tagPoseMap[innerRight]
            val pOR = tagPoseMap[outerRight]

            if (pIL != null && pIR != null) {
                // Both inner tags visible: exact center is the midpoint between inner pair!
                bestX = (pIL.position.x + pIR.position.x) / 2.0
                bestZ = (pIL.position.z + pIR.position.z) / 2.0
                bestTy = ((tagTyMap[innerLeft] ?: 0.0) + (tagTyMap[innerRight] ?: 0.0)) / 2.0
                matchedGroupName = "Group ${groupIndex + 1} (Inner Pair ${innerLeft}&${innerRight})"
                targetFound = true
                break
            } else if (pIL != null) {
                val yaw = PI / 180.0 * pIL.orientation.getYaw(AngleUnit.DEGREES)
                bestX = pIL.position.x + (INNER_TAG_OFFSET_METERS * cos(yaw))
                bestZ = pIL.position.z - (INNER_TAG_OFFSET_METERS * sin(yaw))
                bestTy = tagTyMap[innerLeft] ?: 0.0
                matchedGroupName = "Group ${groupIndex + 1} (Inner Left $innerLeft)"
                targetFound = true
                break
            } else if (pIR != null) {
                val yaw = PI / 180.0 * pIR.orientation.getYaw(AngleUnit.DEGREES)
                bestX = pIR.position.x - (INNER_TAG_OFFSET_METERS * cos(yaw))
                bestZ = pIR.position.z + (INNER_TAG_OFFSET_METERS * kotlin.math.sin(yaw))
                bestTy = tagTyMap[innerRight] ?: 0.0
                matchedGroupName = "Group ${groupIndex + 1} (Inner Right $innerRight)"
                targetFound = true
                break
            } else if (pOL != null) {
                val yaw = PI / 180.0 * pOL.orientation.getYaw(AngleUnit.DEGREES)
                bestX = pOL.position.x + (OUTER_TAG_OFFSET_METERS * cos(yaw))
                bestZ = pOL.position.z - (OUTER_TAG_OFFSET_METERS * sin(yaw))
                bestTy = tagTyMap[outerLeft] ?: 0.0
                matchedGroupName = "Group ${groupIndex + 1} (Outer Left $outerLeft)"
                targetFound = true
                break
            } else if (pOR != null) {
                val yaw = PI / 180.0 * pOR.orientation.getYaw(AngleUnit.DEGREES)
                bestX = pOR.position.x - (OUTER_TAG_OFFSET_METERS * cos(yaw))
                bestZ = pOR.position.z + (OUTER_TAG_OFFSET_METERS * sin(yaw))
                bestTy = tagTyMap[outerRight] ?: 0.0
                matchedGroupName = "Group ${groupIndex + 1} (Outer Right $outerRight)"
                targetFound = true
                break
            }
        }

        if (targetFound) {
            // Calculate angle error in degrees to the calculated hive centerline
            val turnAngleToCenter = Math.toDegrees(Math.atan2(bestX, bestZ))
            lastErrorTx = turnAngleToCenter
            lastErrorTy = bestTy

            if (adjustmentTimer.seconds() >= ADJUSTMENT_INTERVAL) {
                adjustmentTimer.reset()
                val dt = pidTimer.seconds()
                pidTimer.reset()

                if (dt > 0.0 && dt < 2.0) {
                    lastErrorTx = (FILTER_ALPHA * lastErrorTx) + ((1.0 - FILTER_ALPHA) * lastError)

                    if (Math.abs(lastErrorTx) < DEADBAND_DEGREES && !isTagInverted) {
                        lastMotorPower = 0.0
                    } else {
                        val targetPower: Double

                        if (isTagInverted) {
                            val pivotDir = if (lastErrorTx >= 0) 1.0 else -1.0
                            targetPower = PIVOT_POWER * pivotDir
                        } else {
                            var rawP = Kp * lastErrorTx
                            if (INVERT_MOTOR) rawP = -rawP
                            targetPower = rawP.coerceIn(-MAX_POWER, MAX_POWER)
                        }

                        val maxChange = MAX_ACCEL * dt
                        lastMotorPower = lastMotorPower.coerceIn(targetPower - maxChange, targetPower + maxChange)
                        lastMotorPower = lastMotorPower.coerceIn(-MAX_POWER, MAX_POWER)
                    }
                }
                motor.power = lastMotorPower
                lastError = lastErrorTx
            }
        } else {
            lastErrorTx = 0.0
            lastErrorTy = 0.0
            lastMotorPower = 0.0
            motor.power = 0.0
            adjustmentTimer.reset()
            pidTimer.reset()
        }

        packet?.apply {
            put("Active Group Name", matchedGroupName)
            put("Hive Center Error (tx)", lastErrorTx)
            put("Hive Center Error (ty)", lastErrorTy)
            put("3D Distance (m)", lastDistance3D)
            put("Camera Motor Power", lastMotorPower)
            put("Visible Tag Count", lastDetectedTags.size)
            put("Tag Inverted", if (isTagInverted) 1 else 0)
            put("LL FPS", status.fps.toInt())
            put("LL CPU %", status.cpu)
            put("Latency (ms)", totalLatency)

            val canvas = fieldOverlay()
            canvas.setStroke("blue")
            canvas.strokeLine(0.0, -30.0, 0.0, 30.0)

            canvas.setFill("red")
            canvas.fillCircle(lastErrorTx, 0.0, 3.0)
        }
    }

    data class AprilTagTarget(
        val id: Int,
        val txDegrees: Double,
        val distanceInches: Double,
        val isValid: Boolean
    )

    data class EstimatedRobotPose(
        val x: Double,
        val y: Double,
        val headingDeg: Double,
        val matchedCluster: String,
        val confidence: Double,
        val distanceToHiveCenterInches: Double
    )

    /**
     * Estimates the global field position (X, Y in inches) of the robot using the known
     * upright AprilTag clusters on the Hive.
     *
     * @param robotHeadingRad Current heading of the robot in radians (from Pinpoint / Pedro localizer)
     * @return EstimatedRobotPose containing estimated X, Y coordinates, or null if no valid upright tag seen
     */
    fun estimateRobotPose(robotHeadingRad: Double): EstimatedRobotPose? {
        val result = limelight?.latestResult ?: return null
        if (!result.isValid) return null

        val fiducials = result.fiducialResults
        if (fiducials.isNullOrEmpty()) return null

        // Map tag IDs to known upright cluster center positions (in inches on 144" x 144" field)
        // Red Audience (34-37): X=59.7, Y=52.5, Normal faces 270° (South)
        // Red Scoring (30-33):  X=59.7, Y=91.5, Normal faces 90° (North)
        // Blue Audience (38-41): X=84.3, Y=52.5, Normal faces 270° (South)
        // Blue Scoring (42-45):  X=84.3, Y=91.5, Normal faces 90° (North)
        for (fr in fiducials) {
            val id = fr.fiducialId
            val tagFieldCenter: Pair<Double, Double>
            val clusterName: String

            when (id) {
                in 30..33 -> {
                    tagFieldCenter = Pair(59.7, 91.5)
                    clusterName = "RED SCORING"
                }
                in 34..37 -> {
                    tagFieldCenter = Pair(59.7, 52.5)
                    clusterName = "RED AUDIENCE"
                }
                in 38..41 -> {
                    tagFieldCenter = Pair(84.3, 52.5)
                    clusterName = "BLUE AUDIENCE"
                }
                in 42..45 -> {
                    tagFieldCenter = Pair(84.3, 91.5)
                    clusterName = "BLUE SCORING"
                }
                else -> continue
            }

            val pose = fr.targetPoseCameraSpace ?: continue
            val roll = abs(pose.orientation.getRoll(AngleUnit.DEGREES))
            val pitch = abs(pose.orientation.getPitch(AngleUnit.DEGREES))

            // Only localize if cell is upright (roll & pitch are stable, not inverted or in transition)
            if (roll > 45.0 || pitch > 45.0) continue

            // 2D horizontal ground distance in inches
            val distInches = hypot(pose.position.x, pose.position.z) * 39.3701
            if (distInches < 6.0 || distInches > 90.0) continue

            // Absolute bearing angle from robot center to tag
            val txRad = Math.toRadians(fr.targetXDegrees)
            val bearingRad = robotHeadingRad - txRad

            // Invert the vector from the known tag location back to robot center
            val estimatedX = tagFieldCenter.first - distInches * sin(bearingRad)
            val estimatedY = tagFieldCenter.second - distInches * cos(bearingRad)

            // Sanity check within the 144" x 144" field boundary
            if (estimatedX in 5.0..139.0 && estimatedY in 5.0..139.0) {
                val headingDeg = Math.toDegrees(robotHeadingRad)
                // Field center of the entire Hive structure is exactly at (X = 72.0, Y = 72.0)
                val distToCenter = hypot(72.0 - estimatedX, 72.0 - estimatedY)
                return EstimatedRobotPose(
                    x = estimatedX,
                    y = estimatedY,
                    headingDeg = headingDeg,
                    matchedCluster = clusterName,
                    confidence = 1.0 / (1.0 + (distInches / 50.0)),
                    distanceToHiveCenterInches = distToCenter
                )
            }
        }

        return null
    }

    /**
     * Calculates the estimated distance (in inches) from the robot's current position to the physical
     * center of the Hive structure at (X=72.0", Y=72.0") whenever any AprilTag on the Hive is seen.
     */
    fun getDistanceToHiveCenter(robotHeadingRad: Double): Double? {
        val pose = estimateRobotPose(robotHeadingRad)
        return pose?.distanceToHiveCenterInches
    }

    fun getBestAprilTag(): AprilTagTarget? {
        if (limelight?.status == null || limelight?.isConnected == false) return null
        val result = limelight?.latestResult ?: return null
        if (!result.isValid) return null

        val fiducials = result.fiducialResults
        if (fiducials.isNullOrEmpty()) {
            return null
        }

        var bestTag: AprilTagTarget? = null
        var minDistance = Double.MAX_VALUE

        for (fr in fiducials) {
            val pose = fr.targetPoseCameraSpace
            val distInches = if (pose != null) sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z) * 39.3701 else 0.0

            if (distInches > 0.0 && distInches < minDistance) {
                minDistance = distInches
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            } else if (bestTag == null) {
                bestTag = AprilTagTarget(id = fr.fiducialId, txDegrees = fr.targetXDegrees, distanceInches = distInches, isValid = true)
            }
        }

        return bestTag
    }

    fun stop() {
        limelight?.stop()
        cameraMotor?.power = 0.0
    }
}
