package org.firstinspires.ftc.teamcode

import com.pedropathing.drivetrain.DrivePowers
import com.qualcomm.robotcore.hardware.Gamepad

/**
 * DriverControls centralizes all input mapping for Driver 1 (Drivetrain)
 * and Driver 2 (Subsystems).
 *
 * This decouples raw gamepad buttons from OpMode and subsystem execution logic.
 */
class DriverControls(val driver1: GamepadEx, val driver2: GamepadEx) {

    /**
     * Checks if Gamepad 2 is connected/assigned on the Driver Station.
     */
    val isGamepad2Connected: Boolean
        get() = driver2.gamepad.id != Gamepad.ID_UNASSOCIATED

    /**
     * Updates internal button tracking for both drivers once per loop iteration.
     */
    fun update() {
        driver1.update()
        driver2.update()
    }

    // --- Driver 1 Controls (Chassis & Navigation) ---

    /** Normalized drive powers with stick deadband, correct signs, and inversions applied */
    val drivePowers: DrivePowers
        get() {
            val fwd = -driver1.getDeadbandStick(driver1.gamepad.left_stick_y)
            val str = -driver1.getDeadbandStick(driver1.gamepad.left_stick_x)
            val rot = -driver1.getDeadbandStick(driver1.gamepad.right_stick_x)
            return DrivePowers(fwd, str, rot)
        }

    /** Toggle Field-Centric drive mode (Rising edge on Driver 1 'A') */
    val toggleFieldCentric: Boolean get() = driver1.aJustPressed

    /** Zero field heading and odometry position (Rising edge on Driver 1 'BACK') */
    val resetOdometry: Boolean get() = driver1.backJustPressed

    // --- Driver 2 Controls (Intake, Nectar & Pollen Launchers) ---
    // When Gamepad 2 is connected, strictly isolate mechanism controls to Driver 2.
    // Fall back to Driver 1 only when Gamepad 2 is not connected.

    /** Spin up Nectar & Pollen launchers */
    val spinUpLaunchers: Boolean
        get() = if (isGamepad2Connected) driver2.gamepad.a else (driver1.gamepad.a || driver1.gamepad.y)

    /** Right Trigger for Nectar Launcher (Right) feed */
    val nectarLauncherTrigger: Float
        get() = if (isGamepad2Connected) driver2.gamepad.right_trigger else driver1.gamepad.right_trigger
    val rightLauncherTrigger: Float get() = nectarLauncherTrigger

    /** Left Trigger for Pollen Launcher (Left) feed */
    val pollenLauncherTrigger: Float
        get() = if (isGamepad2Connected) driver2.gamepad.left_trigger else driver1.gamepad.left_trigger
    val leftLauncherTrigger: Float get() = pollenLauncherTrigger

    /** Right Bumper for intake forward */
    val intakeForward: Boolean
        get() = if (isGamepad2Connected) driver2.gamepad.right_bumper else driver1.gamepad.right_bumper

    /** Left Bumper for intake reverse (unjam) */
    val intakeReverse: Boolean
        get() = if (isGamepad2Connected) driver2.gamepad.left_bumper else driver1.gamepad.left_bumper

    /** Adjust Nectar Launcher velocity up (Rising edge on D-Pad Up) */
    val nectarVelocityUp: Boolean
        get() = if (isGamepad2Connected) driver2.dpadUpJustPressed else driver1.dpadUpJustPressed
    val launcher1VelocityUp: Boolean get() = nectarVelocityUp

    /** Adjust Nectar Launcher velocity down (Rising edge on D-Pad Down) */
    val nectarVelocityDown: Boolean
        get() = if (isGamepad2Connected) driver2.dpadDownJustPressed else driver1.dpadDownJustPressed
    val launcher1VelocityDown: Boolean get() = nectarVelocityDown

    /** Adjust Pollen Launcher velocity up (Rising edge on D-Pad Right) */
    val pollenVelocityUp: Boolean
        get() = if (isGamepad2Connected) driver2.dpadRightJustPressed else driver1.dpadRightJustPressed
    val launcher2VelocityUp: Boolean get() = pollenVelocityUp

    /** Adjust Pollen Launcher velocity down (Rising edge on D-Pad Left) */
    val pollenVelocityDown: Boolean
        get() = if (isGamepad2Connected) driver2.dpadLeftJustPressed else driver1.dpadLeftJustPressed
    val launcher2VelocityDown: Boolean get() = pollenVelocityDown
}
