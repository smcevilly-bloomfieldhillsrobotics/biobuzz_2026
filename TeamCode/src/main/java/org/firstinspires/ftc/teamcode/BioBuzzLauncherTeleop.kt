package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * A dedicated TeleOp mode focused exclusively on the Intake, Nectar Launcher, and Pollen Launcher subsystems.
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
        // Driver 1: Controls Intake and Launchers (for standalone testing)
        // Button A: Spin up Nectar & Pollen launchers to target velocity
        // Right Trigger: Feed Nectar Launcher (Right)
        // Left Trigger: Feed Pollen Launcher (Left)
        // Right Bumper: Run intake forwards
        // Left Bumper: Reverse intake & servos (unjam)
        intakeLauncher.update(
            gamepad1.a,
            gamepad1.right_trigger,
            gamepad1.left_trigger,
            gamepad1.right_bumper,
            gamepad1.left_bumper
        )

        // Adjust Nectar Launcher Velocity (D-Pad Up/Down)
        if (gamepad1.dpad_up && !lastDpadUp) {
            intakeLauncher.adjustNectarLauncherVelocity(50.0)
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            intakeLauncher.adjustNectarLauncherVelocity(-50.0)
        }
        lastDpadUp = gamepad1.dpad_up
        lastDpadDown = gamepad1.dpad_down

        // Adjust Pollen Launcher Velocity (D-Pad Left/Right)
        if (gamepad1.dpad_right && !lastDpadRight) {
            intakeLauncher.adjustPollenLauncherVelocity(50.0)
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            intakeLauncher.adjustPollenLauncherVelocity(-50.0)
        }
        lastDpadRight = gamepad1.dpad_right
        lastDpadLeft = gamepad1.dpad_left

        val nectarVel = robot.nectarLauncher?.velocity ?: 0.0
        val pollenVel = robot.pollenLauncher?.velocity ?: 0.0

        val packet = TelemetryPacket()
        packet.put("Nectar Launcher Vel", nectarVel)
        packet.put("Nectar Launcher Target", IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        packet.put("Pollen Launcher Vel", pollenVel)
        packet.put("Pollen Launcher Target", IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
        packet.put("Intake Power", intakeLauncher.intakePower)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        // Telemetry
        telemetry.addData("--- BioBuzz Launcher Tuning ---", "")
        telemetry.addData("Controls", "A: Spin Up, RT: Nectar Launcher, LT: Pollen Launcher, RB: Intake, LB: Reverse/Unjam")
        telemetry.addData("Tuning", "D-Pad Up/Down: Nectar Launcher, D-Pad Left/Right: Pollen Launcher")
        telemetry.addData("Nectar Launcher Vel", "%.1f ticks/sec (Target: %.0f)", nectarVel, IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        telemetry.addData("Pollen Launcher Vel", "%.1f ticks/sec (Target: %.0f)", pollenVel, IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)
        
        telemetry.update()
    }

    override fun stop() {
        intakeLauncher.stop()
    }
}
