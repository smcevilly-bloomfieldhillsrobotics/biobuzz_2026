package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs

import com.qualcomm.robotcore.eventloop.opmode.Disabled

/**
 * Diagnostic & Calibration TeleOp powered directly by Pedro Pathing's Mecanum Drivetrain & DrivePowers.
 *
 * Controls & Features:
 * 1. INDIVIDUAL MOTOR ISOLATION (Press and hold each face button):
 *    - (X / Square): Spin ONLY Front-Left (FL) forward
 *    - (Y / Triangle): Spin ONLY Front-Right (FR) forward
 *    - (A / Cross): Spin ONLY Back-Left (BL) forward
 *    - (B / Circle): Spin ONLY Back-Right (BR) forward
 *
 * 2. DIRECTIONAL TRANSLATION CHECKS (D-Pad):
 *    - D-Pad UP: Pure Forward (all 4 wheels spin forward)
 *    - D-Pad DOWN: Pure Backward (all 4 wheels spin backward)
 *    - D-Pad RIGHT: Pure Strafe Right (FR/BL fwd, FL/BR rev)
 *    - D-Pad LEFT: Pure Strafe Left (FL/BR fwd, FR/BL rev)
 *
 * 3. ROTATION CHECKS (Bumpers):
 *    - Left Bumper: Turn Left / CCW (Right side forward, Left side reverse)
 *    - Right Bumper: Turn Right / CW (Left side forward, Right side reverse)
 *
 * 4. FULL KINEMATICS JOYSTICK TEST (Pedro Pathing DrivePowers):
 *    - Left Stick: Translation (Y = forward/back, X = strafe)
 *    - Right Stick X: Rotation
 */
@Disabled
@TeleOp(name = "Mecanum Wheel Diagnostics & Setup", group = "Diagnostics")
class MecanumWheelTestTeleop : LinearOpMode() {

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        pedroMecanum = robot.createPedroMecanum(hardwareMap)

        telemetry.addLine("=== PEDRO MECANUM DIAGNOSTIC OPMODE READY ===")
        telemetry.addLine("Hold X: Test FL | Hold Y: Test FR")
        telemetry.addLine("Hold A: Test BL | Hold B: Test BR")
        telemetry.addLine("D-Pad: Drive directions | Sticks: Pedro DrivePowers")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            var flPower = 0.0
            var frPower = 0.0
            var blPower = 0.0
            var brPower = 0.0
            var activeTest = "None (Idle)"

            // 1. Single Motor Isolation Tests (0.4 power)
            if (gamepad1.x) {
                flPower = 0.4
                activeTest = "ISOLATING Front-Left (FL)"
            } else if (gamepad1.y) {
                frPower = 0.4
                activeTest = "ISOLATING Front-Right (FR)"
            } else if (gamepad1.a) {
                blPower = 0.4
                activeTest = "ISOLATING Back-Left (BL)"
            } else if (gamepad1.b) {
                brPower = 0.4
                activeTest = "ISOLATING Back-Right (BR)"
            }
            // 2. Cardinal Movement Tests via D-Pad
            else if (gamepad1.dpad_up) {
                flPower = 0.4; frPower = 0.4; blPower = 0.4; brPower = 0.4
                activeTest = "FORWARD (All wheels forward)"
            } else if (gamepad1.dpad_down) {
                flPower = -0.4; frPower = -0.4; blPower = -0.4; brPower = -0.4
                activeTest = "BACKWARD (All wheels reverse)"
            } else if (gamepad1.dpad_right) {
                flPower = -0.4; frPower = 0.4; blPower = 0.4; brPower = -0.4
                activeTest = "STRAFE RIGHT (FR/BL fwd, FL/BR rev)"
            } else if (gamepad1.dpad_left) {
                flPower = 0.4; frPower = -0.4; blPower = -0.4; brPower = 0.4
                activeTest = "STRAFE LEFT (FL/BR fwd, FR/BL rev)"
            }
            // 3. Rotation Tests via Bumpers
            else if (gamepad1.left_bumper) {
                flPower = -0.4; frPower = 0.4; blPower = -0.4; brPower = 0.4
                activeTest = "TURN LEFT / CCW (Right fwd, Left rev)"
            } else if (gamepad1.right_bumper) {
                flPower = 0.4; frPower = -0.4; blPower = 0.4; brPower = -0.4
                activeTest = "TURN RIGHT / CW (Left fwd, Right rev)"
            }
            // 4. Normal Holonomic Kinematics via Joysticks (Pedro DrivePowers)
            else {
                val forward = -gamepad1.left_stick_y.toDouble()
                val strafe = -gamepad1.left_stick_x.toDouble() // Negated so gamepad right = right strafe in Pedro Pathing
                val rotate = gamepad1.right_stick_x.toDouble()

                if (abs(forward) > 0.05 || abs(strafe) > 0.05 || abs(rotate) > 0.05) {
                    activeTest = "PEDRO JOYSTICK DRIVE"
                    pedroMecanum.drive(DrivePowers(forward, strafe, rotate), true)
                    val wp = pedroMecanum.wheelPowers
                    flPower = if (wp.isNotEmpty()) wp[0] else 0.0
                    frPower = if (wp.size > 1) wp[1] else 0.0
                    blPower = if (wp.size > 2) wp[2] else 0.0
                    brPower = if (wp.size > 3) wp[3] else 0.0
                } else {
                    pedroMecanum.stop()
                }
            }

            // Apply direct power when running individual motor or cardinal tests
            if (activeTest != "PEDRO JOYSTICK DRIVE") {
                robot.frontLeftDrive?.power = flPower
                robot.frontRightDrive?.power = frPower
                robot.backLeftDrive?.power = blPower
                robot.backRightDrive?.power = brPower
            }

            // Dashboard stream
            val packet = TelemetryPacket()
            packet.put("Active Test", activeTest)
            packet.put("FL Power", flPower)
            packet.put("FR Power", frPower)
            packet.put("BL Power", blPower)
            packet.put("BR Power", brPower)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            // Driver Station telemetry
            telemetry.addData("--- Pedro Mecanum Diagnostics ---", "")
            telemetry.addData("Active Test", activeTest)
            telemetry.addData("FL Power (X)", "%.2f", flPower)
            telemetry.addData("FR Power (Y)", "%.2f", frPower)
            telemetry.addData("BL Power (A)", "%.2f", blPower)
            telemetry.addData("BR Power (B)", "%.2f", brPower)
            telemetry.addLine()
            telemetry.addLine("Physical Mecanum Checklist:")
            telemetry.addLine("1. Roller 'O' shape on top / 'X' shape on bottom")
            telemetry.addLine("2. Each motor spins forward when pressed alone")
            telemetry.addLine("3. D-Pad UP drives cleanly straight forward")
            telemetry.addLine("4. D-Pad RIGHT drives straight right without turning")
            telemetry.update()

            sleep(20)
        }

        pedroMecanum.stop()
    }
}
