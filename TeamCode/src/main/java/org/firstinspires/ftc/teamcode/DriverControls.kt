package org.firstinspires.ftc.teamcode

import com.pedropathing.drivetrain.DrivePowers
import org.firstinspires.ftc.teamcode.nextftc.NextGamepad
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

/**
 * DriverControls centralizes all input mapping for Driver 1 (Drivetrain)
 * and Driver 2 (Subsystems), utilizing NextFTC's NextGamepad.
 *
 * This decouples raw gamepad buttons and fallback logic from the OpMode.
 */
class DriverControls(private val driver1: NextGamepad, private val driver2: NextGamepad) {

    /** Checks if Gamepad 2 is connected/assigned on the Driver Station. */
    val isGamepad2Connected: Boolean
        get() = driver2.gamepad.id != -1

    /** Helper for stick deadbands and exponential sensitivity scaling curves. */
    private fun getDeadbandStick(raw: Float, deadband: Float = 0.05f, exponent: Double = 1.0): Double {
        if (abs(raw) < deadband) return 0.0
        val s = sign(raw.toDouble())
        val scaled = ((abs(raw) - deadband) / (1.0 - deadband))
        return s * abs(scaled).pow(exponent)
    }

    // --- Continuous Polling Controls ---

    /** Normalized drive powers with stick deadband, correct signs, and inversions applied */
    val drivePowers: DrivePowers
        get() {
            val fwd = -getDeadbandStick(driver1.gamepad.left_stick_y)
            val str = -getDeadbandStick(driver1.gamepad.left_stick_x)
            val rot = -getDeadbandStick(driver1.gamepad.right_stick_x)
            return DrivePowers(fwd, str, rot)
        }

    val spinUpLaunchers: Boolean
        get() = if (isGamepad2Connected) driver2.gamepad.a else (driver1.gamepad.a || driver1.gamepad.y)

    val nectarLauncherTrigger: Float
        get() = if (isGamepad2Connected) driver2.gamepad.right_trigger else driver1.gamepad.right_trigger

    val pollenLauncherTrigger: Float
        get() = if (isGamepad2Connected) driver2.gamepad.left_trigger else driver1.gamepad.left_trigger

    val intakeForward: Boolean
        get() = if (isGamepad2Connected) driver2.gamepad.right_bumper else driver1.gamepad.right_bumper

    val intakeReverse: Boolean
        get() = if (isGamepad2Connected) driver2.gamepad.left_bumper else driver1.gamepad.left_bumper

    // --- Discrete Event Bindings ---

    /**
     * Binds one-shot events (like button presses) using NextGamepad's onPress.
     * Automatically handles falling back to Gamepad 1 if Gamepad 2 is disconnected.
     */
    fun bind(
        onToggleFieldCentric: (() -> Unit)? = null,
        onResetOdometry: (() -> Unit)? = null,
        onNectarVelocityUp: (() -> Unit)? = null,
        onNectarVelocityDown: (() -> Unit)? = null,
        onPollenVelocityUp: (() -> Unit)? = null,
        onPollenVelocityDown: (() -> Unit)? = null
    ) {
        onToggleFieldCentric?.let { driver1.a.onPress(it) }
        onResetOdometry?.let { driver1.back.onPress(it) }

        onNectarVelocityUp?.let {
            driver1.dpadUp.onPress { if (!isGamepad2Connected) it() }
            driver2.dpadUp.onPress(it)
        }
        
        onNectarVelocityDown?.let {
            driver1.dpadDown.onPress { if (!isGamepad2Connected) it() }
            driver2.dpadDown.onPress(it)
        }
        
        onPollenVelocityUp?.let {
            driver1.dpadRight.onPress { if (!isGamepad2Connected) it() }
            driver2.dpadRight.onPress(it)
        }
        
        onPollenVelocityDown?.let {
            driver1.dpadLeft.onPress { if (!isGamepad2Connected) it() }
            driver2.dpadLeft.onPress(it)
        }
    }
}
