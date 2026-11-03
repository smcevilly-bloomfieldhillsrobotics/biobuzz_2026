package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs

/**
 * A dedicated TeleOp mode focused exclusively on the Intake, Nectar Launcher, and Pollen Launcher subsystems
 * with full driving capability.
 */
@TeleOp(name = "BioBuzz Launcher Tuning", group = "Test")
class BioBuzzLauncherTeleop : OpMode() {

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var intakeLauncher: IntakeLauncherSubsystem
    private lateinit var controls: DriverControls

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)
        intakeLauncher = IntakeLauncherSubsystem(robot)
        controls = DriverControls(GamepadEx(gamepad1), GamepadEx(gamepad2))

        telemetry.addData("Status", "Launcher Subsystem & Drivetrain Ready (1 or 2 Gamepads)")
        telemetry.update()
    }

    override fun loop() {
        // Update centralized controls (supports single Gamepad 1 or dual Gamepad 1 + 2)
        controls.update()

        // Driver 1: Drive controls
        val powers = controls.drivePowers
        if (abs(powers.forward()) > 0.05 || abs(powers.strafe()) > 0.05 || abs(powers.turn()) > 0.05) {
            pedroMecanum.drive(powers, true)
        } else {
            pedroMecanum.stop()
        }

        // Intake and Launcher Controls (Gamepad 2 or Gamepad 1 fallback)
        // Button A / Y: Spin up Nectar & Pollen launchers
        // Right Trigger: Feed Nectar Launcher (Right)
        // Left Trigger: Feed Pollen Launcher (Left)
        // Right Bumper: Intake Forward
        // Left Bumper: Reverse Intake & Servos (unjam)
        intakeLauncher.update(
            controls.spinUpLaunchers,
            controls.nectarLauncherTrigger,
            controls.pollenLauncherTrigger,
            controls.intakeForward,
            controls.intakeReverse
        )

        // Adjust Nectar Launcher Velocity (D-Pad Up/Down)
        if (controls.nectarVelocityUp) {
            intakeLauncher.adjustNectarLauncherVelocity(50.0)
        }
        if (controls.nectarVelocityDown) {
            intakeLauncher.adjustNectarLauncherVelocity(-50.0)
        }

        // Adjust Pollen Launcher Velocity (D-Pad Left/Right)
        if (controls.pollenVelocityUp) {
            intakeLauncher.adjustPollenLauncherVelocity(50.0)
        }
        if (controls.pollenVelocityDown) {
            intakeLauncher.adjustPollenLauncherVelocity(-50.0)
        }

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
        telemetry.addData("Controls", "A/Y: Spin Up, RT: Nectar Launcher, LT: Pollen Launcher, RB: Intake, LB: Reverse/Unjam")
        telemetry.addData("Tuning", "D-Pad Up/Down: Nectar Launcher, D-Pad Left/Right: Pollen Launcher")
        telemetry.addData("Nectar Launcher Vel", "%.1f ticks/sec (Target: %.0f)", nectarVel, IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY)
        telemetry.addData("Pollen Launcher Vel", "%.1f ticks/sec (Target: %.0f)", pollenVel, IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY)
        telemetry.addData("Intake Power", "%.2f", intakeLauncher.intakePower)
        
        telemetry.update()
    }

    override fun stop() {
        pedroMecanum.stop()
        intakeLauncher.stop()
    }
}
