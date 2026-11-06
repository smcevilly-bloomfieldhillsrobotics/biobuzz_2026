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
import kotlin.math.hypot

@Config
class TelemetrySubsystem(telemetry: Telemetry) {

    private val telemetry: Telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    companion object {
        @JvmField var LIMELIGHT_IP = "http://limelight.local:5800"
        @JvmField var PANELS_PROXY_URL = "http://192.168.43.1:5801"
    }

    fun publishBioBuzzTeleopData(
        fieldCentric: Boolean,
        intakeLauncher: IntakeLauncherSubsystem,
        odometry: OdometrySubsystem,
        aprilTag: LimelightAimingSubsystem.AprilTagTarget?,
        flPower: Double,
        frPower: Double,
        blPower: Double,
        brPower: Double,
        nectarLauncherVel: Double,
        pollenLauncherVel: Double,
        rightServoPos: Double = 0.0,
        leftServoPos: Double = 0.0
    ) {
        val packet = TelemetryPacket()
        packet.put("Drive Mode", if (fieldCentric) "Field-Centric" else "Robot-Centric")
        packet.put("Nectar Launcher Vel", nectarLauncherVel)
        packet.put("Nectar Launcher Target", IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        packet.put("Pollen Launcher Vel", pollenLauncherVel)
        packet.put("Pollen Launcher Target", IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
        packet.put("Intake Power", intakeLauncher.intakePower)
        packet.put("FL Power", flPower)
        packet.put("FR Power", frPower)
        packet.put("BL Power", blPower)
        packet.put("BR Power", brPower)

        packet.put("Right Servo Pos", rightServoPos)
        packet.put("Left Servo Pos", leftServoPos)

        if (aprilTag != null && aprilTag.isValid) {
            packet.put("AprilTag ID", aprilTag.id)
            packet.put("AprilTag Distance (in)", aprilTag.distanceInches)
            packet.put("AprilTag Offset (°)", aprilTag.txDegrees)
        }

        odometry.update(packet)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        telemetry.addData("--- BioBuzz Mecanum Teleop (Pedro) ---", "")
        telemetry.addData("Drive Mode", if (fieldCentric) "Field-Centric (Press A to toggle)" else "Robot-Centric (Press A to toggle)")
        telemetry.addData("Motors (FL, FR, BL, BR)", "%.2f, %.2f, %.2f, %.2f", flPower, frPower, blPower, brPower)
        telemetry.addData("Controls", "A: Spin Up, RT: Nectar Launcher, LT: Pollen Launcher, RB: Intake, LB: Reverse/Unjam")
        telemetry.addData("Nectar Launcher Vel", "%.1f ticks/sec (Target: %.0f)", nectarLauncherVel, IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        telemetry.addData("Pollen Launcher Vel", "%.1f ticks/sec (Target: %.0f)", pollenLauncherVel, IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
        telemetry.addData("Right Servo Pos", "%.2f (Rest: %.2f, Launch: %.2f)", rightServoPos, IntakeLauncherSubsystem.RIGHT_SERVO_REST_POSITION, IntakeLauncherSubsystem.RIGHT_SERVO_LAUNCH_POSITION)
        telemetry.addData("Left Servo Pos", "%.2f (Rest: %.2f, Launch: %.2f)", leftServoPos, IntakeLauncherSubsystem.LEFT_SERVO_REST_POSITION, IntakeLauncherSubsystem.LEFT_SERVO_LAUNCH_POSITION)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)

        if (aprilTag != null && aprilTag.isValid) {
            val tagStr = if (aprilTag.id >= 0) "#${aprilTag.id}" else "Lock"
            telemetry.addData("Limelight AprilTag", "%s | Distance: %.1f in | Offset: %.1f°", tagStr, aprilTag.distanceInches, aprilTag.txDegrees)
        } else {
            telemetry.addData("Limelight AprilTag", "No AprilTag Target Detected")
        }

        telemetry.addData("--- Odometry (Pinpoint / Pedro) ---", "")
        telemetry.addData("X Position (in)", "%.2f", odometry.posX)
        telemetry.addData("Y Position (in)", "%.2f", odometry.posY)
        telemetry.addData("Heading (deg)", "%.2f", odometry.heading)
        telemetry.update()
    }

    fun publishAutoData(
        opModeName: String,
        currentStep: String,
        intakeLauncher: IntakeLauncherSubsystem,
        poseX: Double,
        poseY: Double,
        headingDeg: Double,
        aprilTag: LimelightAimingSubsystem.AprilTagTarget?,
        aprilTagFound: Boolean,
        nectarLauncherVel: Double,
        pollenLauncherVel: Double
    ) {
        val packet = TelemetryPacket()
        packet.put("Step", currentStep)
        packet.put("X", poseX)
        packet.put("Y", poseY)
        packet.put("Heading (deg)", headingDeg)
        packet.put("AprilTag Found", aprilTagFound)
        packet.put("Nectar Launcher Vel", nectarLauncherVel)
        packet.put("Pollen Launcher Vel", pollenLauncherVel)
        packet.put("Intake Power", intakeLauncher.intakePower)

        if (aprilTag != null && aprilTag.isValid) {
            packet.put("AprilTag ID", aprilTag.id)
            packet.put("AprilTag Distance (in)", aprilTag.distanceInches)
            packet.put("AprilTag Offset (°)", aprilTag.txDegrees)
            val distToHive = hypot(72.0 - poseX, 72.0 - poseY)
            packet.put("Dist to Hive Center (in)", distToHive)
        }

        val fieldOverlay = packet.fieldOverlay()
        fieldOverlay.setStroke("#F59E0B")
        fieldOverlay.setStrokeWidth(2)
        
        // FTC Dashboard coordinates: (0,0) is center, +X goes UP, +Y goes LEFT
        // Pedro Pathing coordinates: (0,0) is bottom-left, +X goes UP, +Y goes RIGHT
        // We need to map Pedro (X, Y) to FTC Dashboard (-Y + 72, X - 72)
        val drawX = poseX - 72.0
        val drawY = 72.0 - poseY
        val drawHeading = Math.toRadians(headingDeg) - Math.PI / 2.0 

        fieldOverlay.strokeCircle(drawX, drawY, 9.0)
        
        val length = 12.0
        fieldOverlay.strokeLine(
            drawX,
            drawY,
            drawX + length * Math.cos(drawHeading),
            drawY + length * Math.sin(drawHeading)
        )

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        telemetry.addData("--- $opModeName ---", "")
        telemetry.addData("Step", currentStep)
        telemetry.addData("X Position (in)", "%.2f", poseX)
        telemetry.addData("Y Position (in)", "%.2f", poseY)
        telemetry.addData("Heading (deg)", "%.2f", headingDeg)
        telemetry.addData("AprilTag Found", aprilTagFound)
        telemetry.addData("Nectar Launcher Vel", "%.1f ticks/sec (Target: %.0f)", nectarLauncherVel, IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        telemetry.addData("Pollen Launcher Vel", "%.1f ticks/sec (Target: %.0f)", pollenLauncherVel, IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)

        if (aprilTag != null && aprilTag.isValid) {
            val tagStr = if (aprilTag.id >= 0) "#${aprilTag.id}" else "Lock"
            val distToHive = hypot(72.0 - poseX, 72.0 - poseY)
            telemetry.addData("Limelight AprilTag", "%s | Distance: %.1f in | Offset: %.1f°", tagStr, aprilTag.distanceInches, aprilTag.txDegrees)
            telemetry.addData("Dist to Hive Center", "%.1f inches", distToHive)
        } else {
            telemetry.addData("Limelight AprilTag", "No AprilTag Target Detected")
        }

        telemetry.update()
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
