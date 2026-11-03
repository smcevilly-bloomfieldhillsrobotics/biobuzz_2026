package org.firstinspires.ftc.teamcode.nextftc

import com.qualcomm.robotcore.hardware.Gamepad

/**
 * NextFTC Event-Driven Gamepad (https://nextftc.dev/).
 * Replaces manual edge-detection boilers with high-level event bindings.
 */
class NextGamepad(val gamepad: Gamepad) {

    class ButtonBinding {
        private var lastState = false
        private var onPressHandler: (() -> Unit)? = null
        private var onReleaseHandler: (() -> Unit)? = null
        private var whenHeldHandler: (() -> Unit)? = null

        fun onPress(handler: () -> Unit) {
            onPressHandler = handler
        }

        fun onRelease(handler: () -> Unit) {
            onReleaseHandler = handler
        }

        fun whenHeld(handler: () -> Unit) {
            whenHeldHandler = handler
        }

        fun update(currentState: Boolean) {
            if (currentState && !lastState) {
                onPressHandler?.invoke()
            } else if (!currentState && lastState) {
                onReleaseHandler?.invoke()
            }
            if (currentState) {
                whenHeldHandler?.invoke()
            }
            lastState = currentState
        }
    }

    val a = ButtonBinding()
    val b = ButtonBinding()
    val x = ButtonBinding()
    val y = ButtonBinding()
    val dpadUp = ButtonBinding()
    val dpadDown = ButtonBinding()
    val dpadLeft = ButtonBinding()
    val dpadRight = ButtonBinding()
    val leftBumper = ButtonBinding()
    val rightBumper = ButtonBinding()
    val back = ButtonBinding()

    fun update() {
        a.update(gamepad.a)
        b.update(gamepad.b)
        x.update(gamepad.x)
        y.update(gamepad.y)
        dpadUp.update(gamepad.dpad_up)
        dpadDown.update(gamepad.dpad_down)
        dpadLeft.update(gamepad.dpad_left)
        dpadRight.update(gamepad.dpad_right)
        leftBumper.update(gamepad.left_bumper)
        rightBumper.update(gamepad.right_bumper)
        back.update(gamepad.back)
    }
}
