package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

/**
 * GamepadEx provides a centralized wrapper around FTC's Gamepad.
 * It handles rising-edge detection (single press events), stick deadbands,
 * and sensitivity exponential scaling.
 */
class GamepadEx(val gamepad: Gamepad) {

    // Previous state snapshot for rising-edge detection
    private var prevA = false
    private var prevB = false
    private var prevX = false
    private var prevY = false
    private var prevDpadUp = false
    private var prevDpadDown = false
    private var prevDpadLeft = false
    private var prevDpadRight = false
    private var prevLeftBumper = false
    private var prevRightBumper = false
    private var prevBack = false
    private var prevStart = false

    // Current rising edge state (true ONLY on the frame the button is first pressed)
    var aJustPressed = false; private set
    var bJustPressed = false; private set
    var xJustPressed = false; private set
    var yJustPressed = false; private set
    var dpadUpJustPressed = false; private set
    var dpadDownJustPressed = false; private set
    var dpadLeftJustPressed = false; private set
    var dpadRightJustPressed = false; private set
    var leftBumperJustPressed = false; private set
    var rightBumperJustPressed = false; private set
    var backJustPressed = false; private set
    var startJustPressed = false; private set

    /**
     * Must be called once per loop iteration in TeleOp loop().
     */
    fun update() {
        aJustPressed = gamepad.a && !prevA
        bJustPressed = gamepad.b && !prevB
        xJustPressed = gamepad.x && !prevX
        yJustPressed = gamepad.y && !prevY

        dpadUpJustPressed = gamepad.dpad_up && !prevDpadUp
        dpadDownJustPressed = gamepad.dpad_down && !prevDpadDown
        dpadLeftJustPressed = gamepad.dpad_left && !prevDpadLeft
        dpadRightJustPressed = gamepad.dpad_right && !prevDpadRight

        leftBumperJustPressed = gamepad.left_bumper && !prevLeftBumper
        rightBumperJustPressed = gamepad.right_bumper && !prevRightBumper

        backJustPressed = gamepad.back && !prevBack
        startJustPressed = gamepad.start && !prevStart

        prevA = gamepad.a
        prevB = gamepad.b
        prevX = gamepad.x
        prevY = gamepad.y
        prevDpadUp = gamepad.dpad_up
        prevDpadDown = gamepad.dpad_down
        prevDpadLeft = gamepad.dpad_left
        prevDpadRight = gamepad.dpad_right
        prevLeftBumper = gamepad.left_bumper
        prevRightBumper = gamepad.right_bumper
        prevBack = gamepad.back
        prevStart = gamepad.start
    }

    /**
     * Helper for stick deadbands and exponential sensitivity scaling curves.
     */
    fun getDeadbandStick(raw: Float, deadband: Float = 0.05f, exponent: Double = 1.0): Double {
        if (abs(raw) < deadband) return 0.0
        val sign = sign(raw.toDouble())
        val scaled = ((abs(raw) - deadband) / (1.0 - deadband))
        return sign * scaled.pow(exponent)
    }
}
