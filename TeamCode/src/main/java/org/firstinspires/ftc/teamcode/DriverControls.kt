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

    // --- Driver 2 Controls (Intake, Nectar & Pollen Launchers) ---

    /** Driver 2 Button A to spin up Nectar & Pollen launchers to target velocity */
    val spinUpLaunchers: Boolean get() = driver2.gamepad.a

    /** Driver 2 Right Trigger for Nectar Launcher (Right Launcher) servo feed */
    val nectarLauncherTrigger: Float get() = driver2.gamepad.right_trigger
    val rightLauncherTrigger: Float get() = nectarLauncherTrigger

    /** Driver 2 Left Trigger for Pollen Launcher (Left Launcher) servo feed */
    val pollenLauncherTrigger: Float get() = driver2.gamepad.left_trigger
    val leftLauncherTrigger: Float get() = pollenLauncherTrigger

    /** Driver 2 Right Bumper for intake forward */
    val intakeForward: Boolean get() = driver2.gamepad.right_bumper

    /** Driver 2 Left Bumper for intake reverse */
    val intakeReverse: Boolean get() = driver2.gamepad.left_bumper

    /** Adjust Nectar Launcher velocity up (Rising edge on D-Pad Up) */
    val nectarVelocityUp: Boolean get() = driver2.dpadUpJustPressed
    val launcher1VelocityUp: Boolean get() = nectarVelocityUp

    /** Adjust Nectar Launcher velocity down (Rising edge on D-Pad Down) */
    val nectarVelocityDown: Boolean get() = driver2.dpadDownJustPressed
    val launcher1VelocityDown: Boolean get() = nectarVelocityDown

    /** Adjust Pollen Launcher velocity up (Rising edge on D-Pad Right) */
    val pollenVelocityUp: Boolean get() = driver2.dpadRightJustPressed
    val launcher2VelocityUp: Boolean get() = pollenVelocityUp

    /** Adjust Pollen Launcher velocity down (Rising edge on D-Pad Left) */
    val pollenVelocityDown: Boolean get() = driver2.dpadLeftJustPressed
    val launcher2VelocityDown: Boolean get() = pollenVelocityDown
}
