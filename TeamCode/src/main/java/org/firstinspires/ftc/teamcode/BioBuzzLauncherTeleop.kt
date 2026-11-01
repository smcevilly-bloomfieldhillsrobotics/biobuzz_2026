package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * A dedicated TeleOp mode focused exclusively on the Intake and Launcher subsystem.
 * This can be useful for tuning launcher velocity or diagnosing intake jams without 
 * engaging the drivetrain or odometry systems.
 */
@TeleOp(name = "BioBuzz Launcher Tuning", group = "Test")
class BioBuzzLauncherTeleop : OpMode() {

    private val robot = RobotHardware()
    private lateinit var intakeLauncher: IntakeLauncherSubsystem
    
    // State tracking for edge-detection on D-Pad buttons
    private var lastDpadRight = false
    private var lastDpadLeft = false
    private var lastDpadUp = false
    private var lastDpadDown = false

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // Only initialize required hardware for intake and launcher
        robot.initIntakeAndLauncher(hardwareMap)
        intakeLauncher = IntakeLauncherSubsystem(robot)

        telemetry.addData("Status", "Launcher Subsystem Ready")
        telemetry.update()
    }

    override fun loop() {
        // Driver 1: Controls Intake and Launcher (for standalone testing)
        // Right Trigger: Spin Right Launcher (robot.launcher)
        // Left Trigger: Spin Left Launcher (robot.launcher2)
        // Right Bumper: Run intake forwards
        // Left Bumper: Reverse intake & servos (unjam)
        intakeLauncher.update(
            gamepad1.right_trigger,
            gamepad1.left_trigger,
            gamepad1.right_bumper,
            gamepad1.left_bumper
        )

        // Adjust Launcher 1 Velocity (D-Pad Left/Right)
        if (gamepad1.dpad_right && !lastDpadRight) {
            intakeLauncher.adjustLauncher1Velocity(50.0)
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            intakeLauncher.adjustLauncher1Velocity(-50.0)
        }
        lastDpadRight = gamepad1.dpad_right
        lastDpadLeft = gamepad1.dpad_left

        // Adjust Launcher 2 Velocity (D-Pad Up/Down)
        if (gamepad1.dpad_up && !lastDpadUp) {
            intakeLauncher.adjustLauncher2Velocity(50.0)
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            intakeLauncher.adjustLauncher2Velocity(-50.0)
        }
        lastDpadUp = gamepad1.dpad_up
        lastDpadDown = gamepad1.dpad_down

        val launcherVel = robot.launcher?.velocity ?: 0.0
        val launcher2Vel = robot.launcher2?.velocity ?: 0.0

        val packet = TelemetryPacket()
        packet.put("Launcher 1 Vel", launcherVel)
        packet.put("Launcher 1 Target", IntakeLauncherSubsystem.TARGET_VELOCITY)
        packet.put("Launcher 2 Vel", launcher2Vel)
        packet.put("Launcher 2 Target", IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY)
        packet.put("Intake Power", intakeLauncher.intakePower)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        // Telemetry
        telemetry.addData("--- BioBuzz Launcher Tuning ---", "")
        telemetry.addData("Controls", "RT: Right Launcher, LT: Left Launcher, RB: Intake, LB: Reverse/Unjam")
        telemetry.addData("Tuning", "D-Pad Left/Right: Right Launcher, D-Pad Up/Down: Left Launcher")
        telemetry.addData("Right Launcher Vel", "%.1f ticks/sec (Target: %.0f)", launcherVel, IntakeLauncherSubsystem.TARGET_VELOCITY)
        telemetry.addData("Left Launcher Vel", "%.1f ticks/sec (Target: %.0f)", launcher2Vel, IntakeLauncherSubsystem.LAUNCHER_2_TARGET_VELOCITY)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)
        
        telemetry.update()
    }

    override fun stop() {
        intakeLauncher.stop()
    }
}
