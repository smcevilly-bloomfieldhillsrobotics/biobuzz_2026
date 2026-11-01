/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config

@Config
class IntakeLauncherSubsystem(private val robot: RobotHardware) {

    @Config
    companion object {
        @JvmField var TARGET_VELOCITY = 1550.0
        @JvmField var MIN_VELOCITY = 1500.0
        @JvmField var LAUNCHER_2_TARGET_VELOCITY = 1550.0
        @JvmField var LAUNCHER_2_MIN_VELOCITY = 1500.0
        @JvmField var SERVO_PULSE_DURATION_MS = 200L // 0.2 seconds
    }

    var intakePower: Double = 0.0
        private set

    fun adjustVelocity(delta: Double) {
        TARGET_VELOCITY += delta
        MIN_VELOCITY += delta
        LAUNCHER_2_TARGET_VELOCITY += delta
        LAUNCHER_2_MIN_VELOCITY += delta
    }

    // Track trigger release state: triggers must return below 0.05 to count as a new press
    private var rightTriggerReleased = true
    private var leftTriggerReleased = true

    // Track whether the servo pulse has fired for the current trigger press
    private var rightTriggerFired = false
    private var leftTriggerFired = false

    // End timestamps for 0.2 second (200ms) CRServo activation
    private var leftServoPulseEndTimeMs: Long = 0L
    private var rightServoPulseEndTimeMs: Long = 0L

    fun adjustLauncher1Velocity(delta: Double) {
        TARGET_VELOCITY += delta
        MIN_VELOCITY += delta
    }

    fun adjustLauncher2Velocity(delta: Double) {
        LAUNCHER_2_TARGET_VELOCITY += delta
        LAUNCHER_2_MIN_VELOCITY += delta
    }

    fun update(
        rightTrigger: Float,
        leftTrigger: Float,
        rightBumper: Boolean,
        leftBumper: Boolean
    ) {
        // Intake controls: Right Bumper = intake forward, Left Bumper = intake reverse
        intakePower = when {
            rightBumper -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }

        robot.intake?.let { it.power = intakePower }

        // --- Trigger Press Edge Detection ---
        // Trigger must return below 0.05 to count as a new press
        if (rightTrigger <= 0.05f) {
            rightTriggerReleased = true
            rightTriggerFired = false
        }
        val isNewRightPress = rightTriggerReleased && (rightTrigger > 0.1f)
        if (isNewRightPress) {
            rightTriggerReleased = false
        }

        if (leftTrigger <= 0.05f) {
            leftTriggerReleased = true
            leftTriggerFired = false
        }
        val isNewLeftPress = leftTriggerReleased && (leftTrigger > 0.1f)
        if (isNewLeftPress) {
            leftTriggerReleased = false
        }

        // Launcher controls:
        // Right Trigger = Right Launcher (robot.launcher)
        // Left Trigger = Left Launcher (robot.launcher2)
        val rightTriggerHeld = rightTrigger > 0.1f
        val leftTriggerHeld = leftTrigger > 0.1f

        if (rightTriggerHeld) {
            robot.launcher?.velocity = TARGET_VELOCITY
        } else {
            robot.launcher?.velocity = 0.0
        }

        if (leftTriggerHeld) {
            robot.launcher2?.velocity = LAUNCHER_2_TARGET_VELOCITY
        } else {
            robot.launcher2?.velocity = 0.0
        }

        val currentVel = robot.launcher?.velocity ?: 0.0
        val currentVel2 = robot.launcher2?.velocity ?: 0.0

        val currentTimeMs = System.currentTimeMillis()

        // When Right Launcher hits target speed on this press, run CRServo for 0.2 seconds
        if (rightTriggerHeld && !rightTriggerFired && currentVel > MIN_VELOCITY) {
            rightTriggerFired = true
            rightServoPulseEndTimeMs = currentTimeMs + SERVO_PULSE_DURATION_MS
        }

        // When Left Launcher hits target speed on this press, run CRServo for 0.2 seconds
        if (leftTriggerHeld && !leftTriggerFired && currentVel2 > LAUNCHER_2_MIN_VELOCITY) {
            leftTriggerFired = true
            leftServoPulseEndTimeMs = currentTimeMs + SERVO_PULSE_DURATION_MS
        }

        // Check if servos are actively pulsing
        val isLeftPulsing = currentTimeMs < leftServoPulseEndTimeMs
        val isRightPulsing = currentTimeMs < rightServoPulseEndTimeMs

        // Left continuous rotation servos power
        val leftServoPower = when {
            isLeftPulsing -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }

        // Right continuous rotation servos power
        val rightServoPower = when {
            isRightPulsing -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }

        val windmillPower = if (isLeftPulsing || isRightPulsing) 1.0 else if (leftBumper) -1.0 else 0.0

        // Apply power to continuous rotation servos
        robot.leftServo?.power = leftServoPower
        robot.leftIntakeServo?.power = if (leftServoPower != 0.0) leftServoPower else intakePower

        robot.rightServo?.power = rightServoPower
        robot.rightIntakeServo?.power = if (rightServoPower != 0.0) rightServoPower else intakePower

        robot.windmillServo?.let { it.power = windmillPower }
    }

    fun stop() {
        intakePower = 0.0
        robot.intake?.power = 0.0
        robot.leftIntakeServo?.power = 0.0
        robot.rightIntakeServo?.power = 0.0
        robot.leftServo?.power = 0.0
        robot.rightServo?.power = 0.0
        robot.launcher?.velocity = 0.0
        robot.launcher2?.velocity = 0.0
        robot.windmillServo?.power = 0.0
        rightTriggerReleased = true
        leftTriggerReleased = true
        rightTriggerFired = false
        leftTriggerFired = false
        leftServoPulseEndTimeMs = 0L
        rightServoPulseEndTimeMs = 0L
    }
}
