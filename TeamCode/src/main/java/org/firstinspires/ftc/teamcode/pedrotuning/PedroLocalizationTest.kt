package org.firstinspires.ftc.teamcode.pedrotuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.OdometrySubsystem
import org.firstinspires.ftc.teamcode.RobotHardware
import kotlin.math.cos
import kotlin.math.sin

import com.qualcomm.robotcore.eventloop.opmode.Disabled

/**
 * Pedro Pathing Tuning Step 1: Odometry & Pod Direction Calibration.
 *
 * Use this OpMode to verify:
 * 1. Pushing robot FORWARD -> X Position INCREASES positively.
 * 2. Pushing robot LEFT -> Y Position INCREASES positively.
 * 3. Rotating Counter-Clockwise -> Heading INCREASES positively (0° -> 90° -> 180°).
 * 4. Press BACK on Gamepad 1 to zero odometry at any time.
 */
@Disabled
@TeleOp(name = "1. Pedro Localization Test", group = "Pedro Tuning")
class PedroLocalizationTest : LinearOpMode() {

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var odometry: OdometrySubsystem

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initPinpoint(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)
        odometry = OdometrySubsystem(robot)

        odometry.resetPosition(0.0, 0.0, 0.0)

        telemetry.addData("Status", "Pedro Localization Test Initialized. Press START.")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val forward = -gamepad1.left_stick_y.toDouble()
            val strafe = -gamepad1.left_stick_x.toDouble() // Negated so gamepad right = right strafe in Pedro Pathing
            val turn = gamepad1.right_stick_x.toDouble()

            pedroMecanum.drive(DrivePowers(forward, strafe, turn), true)

            if (gamepad1.back) {
                odometry.resetPosition(0.0, 0.0, 0.0)
            }

            odometry.update(null)

            val packet = TelemetryPacket()
            packet.put("X (in)", odometry.posX)
            packet.put("Y (in)", odometry.posY)
            packet.put("Heading (deg)", odometry.heading)

            val fieldOverlay = packet.fieldOverlay()
            fieldOverlay.setStroke("#3F51B5")
            fieldOverlay.strokeCircle(odometry.posX, odometry.posY, 9.0)
            val headingRad = Math.toRadians(odometry.heading)
            val length = 12.0
            fieldOverlay.strokeLine(
                odometry.posX,
                odometry.posY,
                odometry.posX + (length * cos(headingRad)),
                odometry.posY + (length * sin(headingRad))
            )

            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            telemetry.addData("--- Pedro Localization Test ---", "")
            telemetry.addData("X Position (in)", "%.3f", odometry.posX)
            telemetry.addData("Y Position (in)", "%.3f", odometry.posY)
            telemetry.addData("Heading (deg)", "%.3f", odometry.heading)
            telemetry.addData("Controls", "Left Stick: Drive/Strafe, Right Stick: Turn, BACK: Zero Position")
            telemetry.update()

            sleep(15)
        }

        pedroMecanum.stop()
    }
}
