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
        @JvmField var SERVO_PULSE_DURATION_MS = 200L // 200 milliseconds (0.2s)
    }

    var intakePower: Double = 0.0
        private set

    // Trigger state tracking: Triggers must return to <= 0.05f (reach 0) to re-arm for another press
    private var rightTriggerReleased = true
    private var leftTriggerReleased = true

    // Timestamps when 200ms servo pulse ends
    private var rightServoEndTimeMs = 0L
    private var leftServoEndTimeMs = 0L

    fun adjustVelocity(delta: Double) {
        TARGET_VELOCITY += delta
        MIN_VELOCITY += delta
        LAUNCHER_2_TARGET_VELOCITY += delta
        LAUNCHER_2_MIN_VELOCITY += delta
    }

    fun adjustLauncher1Velocity(delta: Double) {
        TARGET_VELOCITY += delta
        MIN_VELOCITY += delta
    }

    fun adjustLauncher2Velocity(delta: Double) {
        LAUNCHER_2_TARGET_VELOCITY += delta
        LAUNCHER_2_MIN_VELOCITY += delta
    }

    /**
     * Updates intake and launcher controls:
     * - [spinUpLaunchers] (Button A): Spins launchers up to target velocity
     * - [rightTrigger]: Fires right launcher servo for 10ms upon trigger press (if launcher at speed)
     * - [leftTrigger]: Fires left launcher servo for 10ms upon trigger press (if launcher at speed)
     * - [rightBumper]: Intake forward
     * - [leftBumper]: Reverse intake and launcher feed servos (unjam)
     */
    fun update(
        spinUpLaunchers: Boolean,
        rightTrigger: Float,
        leftTrigger: Float,
        rightBumper: Boolean,
        leftBumper: Boolean
    ) {
        // 1. Launchers Spin Up via Button A
        if (spinUpLaunchers) {
            robot.launcher?.velocity = TARGET_VELOCITY
            robot.launcher2?.velocity = LAUNCHER_2_TARGET_VELOCITY
        } else {
            robot.launcher?.velocity = 0.0
            robot.launcher2?.velocity = 0.0
        }

        val currentVel1 = robot.launcher?.velocity ?: 0.0
        val currentVel2 = robot.launcher2?.velocity ?: 0.0

        val rightAtSpeed = currentVel1 >= MIN_VELOCITY
        val leftAtSpeed = currentVel2 >= LAUNCHER_2_MIN_VELOCITY

        // 2. Intake Motor Control (Right Bumper = Forward, Left Bumper = Reverse)
        intakePower = when {
            rightBumper -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }
        robot.intake?.power = intakePower

        val currentTimeMs = System.currentTimeMillis()

        // 3. Right Trigger Edge Detection & 10ms Pulse Triggering
        if (rightTrigger <= 0.05f) {
            rightTriggerReleased = true
        } else if (rightTrigger > 0.1f && rightTriggerReleased) {
            rightTriggerReleased = false
            if (rightAtSpeed) {
                rightServoEndTimeMs = currentTimeMs + SERVO_PULSE_DURATION_MS
            }
        }

        // 4. Left Trigger Edge Detection & 10ms Pulse Triggering
        if (leftTrigger <= 0.05f) {
            leftTriggerReleased = true
        } else if (leftTrigger > 0.1f && leftTriggerReleased) {
            leftTriggerReleased = false
            if (leftAtSpeed) {
                leftServoEndTimeMs = currentTimeMs + SERVO_PULSE_DURATION_MS
            }
        }

        val rightPulsing = currentTimeMs < rightServoEndTimeMs
        val leftPulsing = currentTimeMs < leftServoEndTimeMs

        // 5. Calculate Servo Powers
        val rightServoPower = when {
            rightPulsing -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }

        val leftServoPower = when {
            leftPulsing -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }

        // Apply powers to left and right launcher feeder servos
        robot.leftLauncherServo?.power = leftServoPower
        robot.rightLauncherServo?.power = rightServoPower
    }

    fun stop() {
        intakePower = 0.0
        robot.intake?.power = 0.0
        robot.leftLauncherServo?.power = 0.0
        robot.rightLauncherServo?.power = 0.0
        robot.launcher?.velocity = 0.0
        robot.launcher2?.velocity = 0.0
        rightTriggerReleased = true
        leftTriggerReleased = true
        rightServoEndTimeMs = 0L
        leftServoEndTimeMs = 0L
    }
}
