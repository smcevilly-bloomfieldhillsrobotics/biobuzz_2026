package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.revhub.drivetrains.Mecanum
import org.firstinspires.ftc.teamcode.nextftc.NextOpMode
import kotlin.math.pow
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.pedropathing.drivetrain.DrivePowers
import kotlin.math.abs
import kotlin.math.sign

/**
 * A dedicated TeleOp mode focused exclusively on the Intake, Nectar Launcher, and Pollen Launcher subsystems
 * with full driving capability.
 */
@TeleOp(name = "BioBuzz Launcher Tuning", group = "Test")
class BioBuzzLauncherTeleop : NextOpMode() {

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var intakeLauncher: IntakeLauncherSubsystem

    override fun onInit() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)
        intakeLauncher = IntakeLauncherSubsystem(robot)

        val dpadUpBinding = if (driver2.gamepad.id != -1) driver2.dpadUp else driver1.dpadUp
        dpadUpBinding.onPress { intakeLauncher.adjustNectarLauncherVelocity(50.0) }
        
        val dpadDownBinding = if (driver2.gamepad.id != -1) driver2.dpadDown else driver1.dpadDown
        dpadDownBinding.onPress { intakeLauncher.adjustNectarLauncherVelocity(-50.0) }
        
        val dpadRightBinding = if (driver2.gamepad.id != -1) driver2.dpadRight else driver1.dpadRight
        dpadRightBinding.onPress { intakeLauncher.adjustPollenLauncherVelocity(50.0) }
        
        val dpadLeftBinding = if (driver2.gamepad.id != -1) driver2.dpadLeft else driver1.dpadLeft
        dpadLeftBinding.onPress { intakeLauncher.adjustPollenLauncherVelocity(-50.0) }

        telemetry.addData("Status", "Launcher Subsystem & Drivetrain Ready (1 or 2 Gamepads)")
        telemetry.update()
    }

    // Helper for input deadband
    private fun getDeadbandStick(raw: Float, deadband: Float = 0.05f, exponent: Double = 1.0): Double {
        if (abs(raw) < deadband) return 0.0
        val sign = sign(raw.toDouble())
        val scaled = ((abs(raw) - deadband) / (1.0 - deadband))
        return sign * abs(scaled).pow(exponent)
    }

    override fun onLoop() {
        val fwdCmd = -getDeadbandStick(driver1.gamepad.left_stick_y)
        val strCmd = -getDeadbandStick(driver1.gamepad.left_stick_x)
        val rotCmd = -getDeadbandStick(driver1.gamepad.right_stick_x)

        if (abs(fwdCmd) > 0.05 || abs(strCmd) > 0.05 || abs(rotCmd) > 0.05) {
            pedroMecanum.drive(DrivePowers(fwdCmd, strCmd, rotCmd), true)
        } else {
            pedroMecanum.stop()
        }

        val isGamepad2Connected = driver2.gamepad.id != -1
        val spinUpLaunchers = if (isGamepad2Connected) driver2.gamepad.a else (driver1.gamepad.a || driver1.gamepad.y)
        val nectarLauncherTrigger = if (isGamepad2Connected) driver2.gamepad.right_trigger else driver1.gamepad.right_trigger
        val pollenLauncherTrigger = if (isGamepad2Connected) driver2.gamepad.left_trigger else driver1.gamepad.left_trigger
        val intakeForward = if (isGamepad2Connected) driver2.gamepad.right_bumper else driver1.gamepad.right_bumper
        val intakeReverse = if (isGamepad2Connected) driver2.gamepad.left_bumper else driver1.gamepad.left_bumper
        intakeLauncher.update(
            spinUpLaunchers,
            nectarLauncherTrigger,
            pollenLauncherTrigger,
            intakeForward,
            intakeReverse
        )

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
