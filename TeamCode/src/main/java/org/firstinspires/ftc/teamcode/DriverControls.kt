package org.firstinspires.ftc.teamcode

import com.pedropathing.drivetrain.DrivePowers

/**
 * DriverControls centralizes all input mapping for Driver 1 (Drivetrain)
 * and Driver 2 (Subsystems).
 *
 * This decouples raw gamepad buttons from OpMode and subsystem execution logic.
 */
class DriverControls(val driver1: GamepadEx, val driver2: GamepadEx) {

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

    // --- Driver 2 Controls (Intake & Dual Launchers) ---

    /** Driver 2 Right Trigger for Right Launcher spin-up */
    val rightLauncherTrigger: Float get() = driver2.gamepad.right_trigger

    /** Driver 2 Left Trigger for Left Launcher spin-up */
    val leftLauncherTrigger: Float get() = driver2.gamepad.left_trigger

    /** Driver 2 Right Bumper for intake forward */
    val intakeForward: Boolean get() = driver2.gamepad.right_bumper

    /** Driver 2 Left Bumper for intake reverse */
    val intakeReverse: Boolean get() = driver2.gamepad.left_bumper

    /** Adjust Right Launcher velocity up (Rising edge on D-Pad Up) */
    val launcher1VelocityUp: Boolean get() = driver2.dpadUpJustPressed

    /** Adjust Right Launcher velocity down (Rising edge on D-Pad Down) */
    val launcher1VelocityDown: Boolean get() = driver2.dpadDownJustPressed

    /** Adjust Left Launcher velocity up (Rising edge on D-Pad Right) */
    val launcher2VelocityUp: Boolean get() = driver2.dpadRightJustPressed

    /** Adjust Left Launcher velocity down (Rising edge on D-Pad Left) */
    val launcher2VelocityDown: Boolean get() = driver2.dpadLeftJustPressed
}
