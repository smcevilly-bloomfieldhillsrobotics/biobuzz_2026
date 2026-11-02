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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.cos
import kotlin.math.sin

/**
 * OdometrySubsystem manages the goBILDA Pinpoint Odometry Computer for precise 
 * field position (X, Y) and heading tracking.
 */
@Config
class OdometrySubsystem(private val robot: RobotHardware) {

    companion object {
        // Pod offsets for goBILDA Pinpoint Odometry Computer:
        // X Pod (Forward pod): Centered on X axis = 0.0 mm
        // Y Pod (Strafe pod): 8.0 inches Backward from center = -203.2 mm
        const val X_POD_OFFSET_MM = 0.0
        const val Y_POD_OFFSET_MM = -203.2
    }

    var posX = 0.0
        private set
    var posY = 0.0
        private set
    var heading = 0.0
        private set
    var isInitialized = false
        private set

    init {
        isInitialized = robot.pinpoint != null
    }

    /**
     * Force an immediate read of the position so posX, posY, and heading are immediately populated
     * rather than waiting for the first external update() call.
     */
    fun refreshOnce() {
        val pinpoint = robot.pinpoint ?: return
        pinpoint.update()
        val pose = pinpoint.position
        posX = pose.getX(DistanceUnit.INCH)
        posY = pose.getY(DistanceUnit.INCH)
        heading = pose.getHeading(AngleUnit.DEGREES)
    }

    fun update(packet: TelemetryPacket?) {
        val pinpoint = robot.pinpoint ?: return
        pinpoint.update()

        val pose = pinpoint.position
        posX = pose.getX(DistanceUnit.INCH)
        posY = pose.getY(DistanceUnit.INCH)
        heading = pose.getHeading(AngleUnit.DEGREES)

        packet?.apply {
            put("Odometry X (in)", posX)
            put("Odometry Y (in)", posY)
            put("Odometry Heading (deg)", heading)

            // Draw robot on field overlay for FTC Dashboard field view
            val fieldOverlay = fieldOverlay()
            fieldOverlay.setStroke("#3F51B5")
            fieldOverlay.strokeCircle(posX, posY, 9.0)
            val headingRad = Math.toRadians(heading)
            val length = 12.0
            fieldOverlay.strokeLine(posX, posY, posX + (length * cos(headingRad)), posY + (length * sin(headingRad)))
        }
    }

    fun resetPosition(x: Double = 0.0, y: Double = 0.0, headingDeg: Double = 0.0) {
        val pinpoint = robot.pinpoint ?: return

        // Wait up to 1 second for Pinpoint IMU calibration/reset to finish before setting pose
        val startWait = System.currentTimeMillis()
        while (pinpoint.deviceStatus == GoBildaPinpointDriver.DeviceStatus.CALIBRATING ||
               pinpoint.deviceStatus == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            if (System.currentTimeMillis() - startWait > 1000) break
            try {
                Thread.sleep(20)
            } catch (_: InterruptedException) {
                break
            }
            pinpoint.update()
        }

        pinpoint.setPosition(Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg))
        pinpoint.update()
        refreshOnce()
    }
}
