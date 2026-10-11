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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "BioBuzz StarterBot Teleop", group = "StarterBot")
class BioBuzzStarterBotTeleop : OpMode() {

    private val robot = RobotHardware()
    private lateinit var drivetrain: DrivetrainSubsystem
    private lateinit var intakeLauncher: IntakeLauncherSubsystem
    private lateinit var odometry: OdometrySubsystem

    override fun init() {
        robot.init(hardwareMap)
        drivetrain = DrivetrainSubsystem(robot)
        intakeLauncher = IntakeLauncherSubsystem(robot)
        odometry = OdometrySubsystem(robot)

        // Reset odometry position upon initialization
        odometry.resetPosition(0.0, 0.0, 0.0)

        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Driver 1: Mobility (Arcade Drive)
        drivetrain.arcadeDrive(-gamepad1.left_stick_y.toDouble(), gamepad1.right_stick_x.toDouble())

        // Quick Odometry Reset (Press BACK on Gamepad 1 to recalibrate/zero field position)
        if (gamepad1.back) {
            odometry.resetPosition(0.0, 0.0, 0.0)
        }

        // Driver 2: Intake & Launcher
        intakeLauncher.update(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.right_bumper)

        // Update Odometry tracking and stream to FTC Dashboard field view
        val packet = TelemetryPacket()
        odometry.update(packet)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        // Telemetry
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", drivetrain.leftPower, drivetrain.rightPower)
        telemetry.addData("Launcher Vel", "%.1f", robot.launcher?.velocity ?: 0.0)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)
        telemetry.addData("--- Odometry ---", "")
        telemetry.addData("X Position (in)", "%.2f", odometry.posX)
        telemetry.addData("Y Position (in)", "%.2f", odometry.posY)
        telemetry.addData("Heading (deg)", "%.2f", odometry.heading)
        telemetry.update()
    }

    override fun stop() {
        drivetrain.stop()
        intakeLauncher.stop()
    }
}
