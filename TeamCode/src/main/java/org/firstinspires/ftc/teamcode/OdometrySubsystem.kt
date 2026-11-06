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

import org.firstinspires.ftc.teamcode.nextftc.NextSubsystem

/**
 * OdometrySubsystem manages the goBILDA Pinpoint Odometry Computer for precise 
 * field position (X, Y) and heading tracking.
 */
@Config
class OdometrySubsystem(private val robot: RobotHardware) : NextSubsystem() {

    companion object {
        // Pod offsets for goBILDA Pinpoint Odometry Computer:
        // X Pod (Forward pod): 8.0 inches left from center = +203.2 mm
        // Y Pod (Strafe pod): 1.5 inches Backward from center = -38.1 mm
        const val X_POD_OFFSET_MM = 203.2
        const val Y_POD_OFFSET_MM = -38.1
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

            // Draw Robot Overlay on FTC Dashboard
            // Dashboard coordinates: (0,0) is center, +X goes UP, +Y goes LEFT
            // Odometry coordinates: (0,0) is bottom-left, +X goes UP, +Y goes RIGHT
            val drawX = posX - 72.0
            val drawY = 72.0 - posY
            val drawHeading = Math.toRadians(heading) - Math.PI / 2.0
            
            val overlay = fieldOverlay()
            overlay.setStroke("#22C55E")
            overlay.setStrokeWidth(2)
            overlay.strokeCircle(drawX, drawY, 9.0)

            val length = 12.0
            overlay.strokeLine(drawX, drawY, drawX + (length * cos(drawHeading)), drawY + (length * sin(drawHeading)))
        }
    }

    fun resetPosition(x: Double = 0.0, y: Double = 0.0, headingDeg: Double = 0.0) {
        val pinpoint = robot.pinpoint ?: return

        try {
            // Non-blocking quick update check for Pinpoint IMU calibration/reset
            if (pinpoint.deviceStatus == GoBildaPinpointDriver.DeviceStatus.CALIBRATING ||
                pinpoint.deviceStatus == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
                pinpoint.update()
            }

            pinpoint.setPosition(Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg))
            pinpoint.update()
            refreshOnce()
        } catch (_: Exception) {
            // Protect against I2C bus timeout/stall
        }
    }
}
