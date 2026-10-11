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

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * LimelightPanTest specialized for 4-tag groups where the exact center 
 * is always located between the inner pair (e.g., between tags 31 & 32, 35 & 36, etc.).
 */
@TeleOp(name = "Limelight Pan Tracking Test", group = "Test")
class LimelightPanTest : LinearOpMode() {

    private var limelight: Limelight3A? = null
    private var panMotor: DcMotor? = null

    // Exact physical offsets from individual tag center to the group centerline (in meters)
    // Tag width = 2.75 in. Centerline is in the 1.0 in gap between inner tags (31 & 32).
    // Inner tag center is 1.875 inches (0.047625 meters) from centerline.
    private val innerTagOffsetMeters = 0.047625
    // Outer tag center offset from centerline (1.0 in gap + 2.75 in tag width + half tag width = ~5.125 in = 0.13017 meters)
    private val outerTagOffsetMeters = 0.13017
    
    private var kP = 0.02

    override fun runOpMode() {
        try {
            limelight = hardwareMap[Limelight3A::class.java, "limelight"]
        } catch (_: Exception) {
            telemetry.addData("Warning", "Limelight not found in hardware map!")
        }

        try {
            panMotor = hardwareMap[DcMotor::class.java, "camera_motor"]
        } catch (_: Exception) {
            try {
                panMotor = hardwareMap[DcMotor::class.java, "panMotor"]
            } catch (_: Exception) {}
        }

        panMotor?.apply {
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        limelight?.pipelineSwitch(0)
        limelight?.start()

        telemetry.addLine("Ready. Point camera near tag groups (30-45).")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val result = limelight?.latestResult
            var targetFound = false

            if ((result != null) && result.isValid) {
                val fiducialResults = result.fiducialResults
                if (fiducialResults != null) {
                    // Map tag ID to its 3D camera-space pose
                    val tagPoseMap = mutableMapOf<Int, Pose3D>()
                    for (tag in fiducialResults) {
                        if ((tag.fiducialId in 30..45)) {
                            tagPoseMap[tag.fiducialId] = tag.targetPoseCameraSpace
                        }
                    }

                    // Check the 4 groups of tags:
                    // Group 0: 30, 31, 32, 33 (Center between 31 & 32)
                    // Group 1: 34, 35, 36, 37 (Center between 35 & 36)
                    // Group 2: 38, 39, 40, 41 (Center between 39 & 40)
                    // Group 3: 42, 43, 44, 45 (Center between 43 & 44)
                    var bestX = 0.0
                    var bestZ = 0.0
                    var matchedGroupName = "None"

                    for (groupIndex in 0..3) {
                        val baseId = 30 + (groupIndex * 4)
                        val outerLeft = baseId      // e.g. 30, 34, 38, 42
                        val innerLeft = baseId + 1  // e.g. 31, 35, 39, 43
                        val innerRight = baseId + 2 // e.g. 32, 36, 40, 44
                        val outerRight = baseId + 3 // e.g. 33, 37, 41, 45

                        val pOL = tagPoseMap[outerLeft]
                        val pIL = tagPoseMap[innerLeft]
                        val pIR = tagPoseMap[innerRight]
                        val pOR = tagPoseMap[outerRight]

                        if (pIL != null && pIR != null) {
                            // Both inner tags visible: exact center is the midpoint between 31 & 32!
                            val centerX = (pIL.position.x + pIR.position.x) / 2.0
                            val centerZ = (pIL.position.z + pIR.position.z) / 2.0
                            bestX = centerX
                            bestZ = centerZ
                            matchedGroupName = "Group ${groupIndex + 1} (Inner Pair $innerLeft&$innerRight)"
                            targetFound = true
                            break
                        } else if (pIL != null) {
                            // Only inner left tag visible (e.g., tag 31)
                            val yaw = Math.toRadians(pIL.orientation.getYaw(AngleUnit.DEGREES))
                            bestX = pIL.position.x + (innerTagOffsetMeters * cos(yaw))
                            bestZ = pIL.position.z - (innerTagOffsetMeters * sin(yaw))
                            matchedGroupName = "Group ${groupIndex + 1} (Inner Left $innerLeft)"
                            targetFound = true
                            break
                        } else if (pIR != null) {
                            // Only inner right tag visible (e.g., tag 32)
                            val yaw = Math.toRadians(pIR.orientation.getYaw(AngleUnit.DEGREES))
                            bestX = pIR.position.x - (innerTagOffsetMeters * cos(yaw))
                            bestZ = pIR.position.z + (innerTagOffsetMeters * sin(yaw))
                            matchedGroupName = "Group ${groupIndex + 1} (Inner Right $innerRight)"
                            targetFound = true
                            break
                        } else if (pOL != null) {
                            // Only outer left visible
                            val yaw = Math.toRadians(pOL.orientation.getYaw(AngleUnit.DEGREES))
                            bestX = pOL.position.x + (outerTagOffsetMeters * cos(yaw))
                            bestZ = pOL.position.z - (outerTagOffsetMeters * sin(yaw))
                            matchedGroupName = "Group ${groupIndex + 1} (Outer Left $outerLeft)"
                            targetFound = true
                            break
                        } else if (pOR != null) {
                            // Only outer right visible
                            val yaw = Math.toRadians(pOR.orientation.getYaw(AngleUnit.DEGREES))
                            bestX = pOR.position.x - (outerTagOffsetMeters * cos(yaw))
                            bestZ = pOR.position.z + (outerTagOffsetMeters * sin(yaw))
                            matchedGroupName = "Group ${groupIndex + 1} (Outer Right $outerRight)"
                            targetFound = true
                            break
                        }
                    }

                    if (targetFound) {
                        // Calculate angle error to the group centerline
                        val turnAngleToCenter = Math.toDegrees(atan2(bestX, bestZ))
                        
                        var motorPower = turnAngleToCenter * kP
                        motorPower = motorPower.coerceIn(-0.3, 0.3)
                        
                        panMotor?.power = motorPower

                        telemetry.addData("Target Group", matchedGroupName)
                        telemetry.addData("Center Angle Error", "%.2f degrees", turnAngleToCenter)
                        telemetry.addData("Motor Power", "%.2f", motorPower)
                    }
                }
            }

            if (!targetFound) {
                panMotor?.power = 0.0
                telemetry.addLine("No target group found. Motor stopped.")
            }

            telemetry.update()
        }
        
        limelight?.stop()
    }
}
