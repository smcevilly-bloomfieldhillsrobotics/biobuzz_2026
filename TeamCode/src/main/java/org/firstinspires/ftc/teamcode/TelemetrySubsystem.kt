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

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class TelemetrySubsystem(telemetry: Telemetry) {

    private val telemetry: Telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    companion object {
        @JvmField var LIMELIGHT_IP = "http://limelight.local:5800"
        @JvmField var PANELS_PROXY_URL = "http://192.168.43.1:5801"
    }

    fun publishInitStatus(isInitialized: Boolean) {
        telemetry.addData("Camera Init Status", if (isInitialized) "SUCCESS (Connected)" else "FAILED")
        telemetry.addData("Stream URL", LIMELIGHT_IP)
        telemetry.update()
    }

    fun publishLoopData(aimingSubsystem: LimelightAimingSubsystem) {
        val packet = TelemetryPacket()
        packet.put("Panels Limelight Proxy Port", PANELS_PROXY_URL)
        packet.put("Stream URL", "$LIMELIGHT_IP/stream.mjpg")

        val initialized = aimingSubsystem.isInitialized
        telemetry.addData("1. Camera Initialized", if (initialized) "YES" else "NO")
        packet.put("Camera Initialized", if (initialized) 1 else 0)

        if (!initialized) {
            telemetry.addData("Error", "Limelight is NOT plugged in or not responding!")
            telemetry.update()
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            return
        }

        aimingSubsystem.limelight?.status?.let { status ->
            telemetry.addData("LL FPS", "%d", status.fps.toInt())
            telemetry.addData("LL CPU Usage", "%.1f%%", status.cpu)
            packet.put("LL FPS", status.fps.toInt())
            packet.put("LL CPU %", status.cpu)
        }

        val visibleTags = aimingSubsystem.lastDetectedTags.toTypedArray()
        val activeGroupName = aimingSubsystem.matchedGroupName
        val errorTx = aimingSubsystem.lastErrorTx
        val motorPower = aimingSubsystem.lastMotorPower

        telemetry.addData("2. All Visible Tags", visibleTags.contentToString())
        telemetry.addData("3. Active Group", activeGroupName)

        if (activeGroupName != "None") {
            telemetry.addData("4. Target Status", "$activeGroupName DETECTED - Aiming")
            telemetry.addData("Hive Center Error (tx)", "%.2f deg", errorTx)
            telemetry.addData("Motor Power", "%.2f", motorPower)

            packet.put("Target Status", "$activeGroupName FOUND")
            packet.put("Active Group Name", activeGroupName)
            packet.put("Hive Center Error (tx)", errorTx)
            packet.put("Camera Motor Power", motorPower)
        } else {
            telemetry.addData("4. Target Status", "NO APRILTAG GROUP IN VIEW")
            telemetry.addData("Motor Power", "0.00 (Stopped)")

            packet.put("Target Status", "NO VALID GROUP")
            packet.put("Active Group Name", "None")
            packet.put("Hive Center Error (tx)", 0)
            packet.put("Camera Motor Power", 0)
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
        telemetry.update()
    }
}
