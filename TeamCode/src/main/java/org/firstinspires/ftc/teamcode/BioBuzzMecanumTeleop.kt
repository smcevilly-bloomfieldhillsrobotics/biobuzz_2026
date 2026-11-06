package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.pow

import org.firstinspires.ftc.teamcode.nextftc.NextOpMode
import kotlin.math.abs
import kotlin.math.sign

/**
 * BioBuzz Mecanum TeleOp powered directly by Pedro Pathing's Mecanum Drivetrain & Pinpoint Odometry.
 * Features full holonomic movement (forward, strafe, turn) and optional field-centric drive.
 */
@TeleOp(name = "BioBuzz Mecanum Teleop (Pedro)", group = "StarterBot")
class BioBuzzMecanumTeleop : NextOpMode() {

    private val robot = RobotHardware()
    private lateinit var pedroMecanum: Mecanum
    private lateinit var intakeLauncher: IntakeLauncherSubsystem
    private lateinit var odometry: OdometrySubsystem
    private lateinit var telemetrySubsystem: TelemetrySubsystem
    private lateinit var aimingSubsystem: LimelightAimingSubsystem

    private lateinit var controls: DriverControls

    // Field-centric vs Robot-centric toggle
    private var fieldCentric = true

    override fun onInit() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.initMecanumDrivetrain(hardwareMap)
        robot.initIntakeAndLauncher(hardwareMap)
        robot.initLimelight(hardwareMap)
        robot.initPinpoint(hardwareMap)

        pedroMecanum = robot.createPedroMecanum(hardwareMap)

        intakeLauncher = IntakeLauncherSubsystem(robot)
        odometry = OdometrySubsystem(robot)
        aimingSubsystem = LimelightAimingSubsystem(hardwareMap)
        telemetrySubsystem = TelemetrySubsystem(telemetry)
        controls = DriverControls(driver1, driver2)

        odometry.resetPosition(0.0, 0.0, 0.0)

        // Bind driver controls to NextGamepad events via DriverControls
        controls.bind(
            onToggleFieldCentric = { fieldCentric = !fieldCentric },
            onResetOdometry = { odometry.resetPosition(0.0, 0.0, 0.0) },
            onNectarVelocityUp = { intakeLauncher.adjustNectarLauncherVelocity(50.0) },
            onNectarVelocityDown = { intakeLauncher.adjustNectarLauncherVelocity(-50.0) },
            onPollenVelocityUp = { intakeLauncher.adjustPollenLauncherVelocity(50.0) },
            onPollenVelocityDown = { intakeLauncher.adjustPollenLauncherVelocity(-50.0) },
            onPulseDurationUp = { intakeLauncher.adjustPulseDuration(5L) },
            onPulseDurationDown = { intakeLauncher.adjustPulseDuration(-5L) },
            onLaunchPositionUp = { intakeLauncher.adjustLaunchPosition(0.05) },
            onLaunchPositionDown = { intakeLauncher.adjustLaunchPosition(-0.05) }
        )

        telemetry.addData("Status", "Pedro Mecanum TeleOp Initialized")
        telemetry.update()
    }

    override fun onLoop() {
        val powers = controls.drivePowers
        
        if (fieldCentric) {
            val headingRad = Math.toRadians(odometry.heading)
            val cosH = cos(headingRad)
            val sinH = sin(headingRad)

            val robotFwd = powers.forward() * cosH + powers.strafe() * sinH
            val robotStr = -powers.forward() * sinH + powers.strafe() * cosH

            pedroMecanum.drive(DrivePowers(robotFwd, robotStr, powers.turn()), true)
        } else {
            pedroMecanum.drive(powers, true)
        }

        // Driver 2: Dual Launcher & Intake controls
        intakeLauncher.update(
            controls.spinUpLaunchers,
            controls.nectarLauncherTrigger,
            controls.pollenLauncherTrigger,
            controls.intakeForward,
            controls.intakeReverse
        )

        // Update Odometry and stream telemetry to FTC Dashboard
        val wheelPowers = pedroMecanum.wheelPowers
        val flPower = if (wheelPowers.isNotEmpty()) wheelPowers[0] else 0.0
        val frPower = if (wheelPowers.size > 1) wheelPowers[1] else 0.0
        val blPower = if (wheelPowers.size > 2) wheelPowers[2] else 0.0
        val brPower = if (wheelPowers.size > 3) wheelPowers[3] else 0.0

        // Limelight AprilTag Detection & Motor aiming
        aimingSubsystem.update(null)
        val aprilTag = aimingSubsystem.getBestAprilTag()
        
        telemetrySubsystem.publishBioBuzzTeleopData(
            fieldCentric,
            intakeLauncher,
            odometry,
            aprilTag,
            flPower, frPower, blPower, brPower,
            robot.nectarLauncher?.velocity ?: 0.0,
            robot.pollenLauncher?.velocity ?: 0.0,
            robot.nectarLauncherServo?.position ?: 0.0,
            robot.pollenLauncherServo?.position ?: 0.0
        )
    }



    // We cannot override stop in NextOpMode because LinearOpMode stop is final

}
