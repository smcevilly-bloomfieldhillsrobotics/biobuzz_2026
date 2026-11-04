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

import org.firstinspires.ftc.teamcode.nextftc.NextSubsystem

@Config
class IntakeLauncherSubsystem(private val robot: RobotHardware) : NextSubsystem() {

    @Config
    companion object {
        // Pollen Launcher (Launcher 1) Target & Min Velocities
        @JvmField var POLLEN_TARGET_VELOCITY = 1550.0
        @JvmField var POLLEN_MIN_VELOCITY = 1500.0

        // Nectar Launcher (Launcher 2) Target & Min Velocities
        @JvmField var NECTAR_TARGET_VELOCITY = 1550.0
        @JvmField var NECTAR_MIN_VELOCITY = 1500.0

        @JvmField var SERVO_PULSE_DURATION_MS = 600L // 600 milliseconds (0.6s)

        // Backward compatibility aliases
        @JvmField var TARGET_VELOCITY = 1550.0
        @JvmField var MIN_VELOCITY = 1500.0
        @JvmField var LAUNCHER_2_TARGET_VELOCITY = 1550.0
        @JvmField var LAUNCHER_2_MIN_VELOCITY = 1500.0
    }

    var intakePower: Double = 0.0
        private set

    // Trigger state tracking: Triggers must return to <= 0.05f (reach 0) to re-arm for another press
    private var nectarTriggerReleased = true
    private var pollenTriggerReleased = true

    // Timestamps when 200ms servo pulse ends
    private var nectarServoEndTimeMs = 0L
    private var pollenServoEndTimeMs = 0L

    fun adjustVelocity(delta: Double) {
        NECTAR_TARGET_VELOCITY += delta
        NECTAR_MIN_VELOCITY += delta
        POLLEN_TARGET_VELOCITY += delta
        POLLEN_MIN_VELOCITY += delta
    }

    fun adjustNectarLauncherVelocity(delta: Double) {
        NECTAR_TARGET_VELOCITY += delta
        NECTAR_MIN_VELOCITY += delta
    }

    fun adjustPollenLauncherVelocity(delta: Double) {
        POLLEN_TARGET_VELOCITY += delta
        POLLEN_MIN_VELOCITY += delta
    }

    fun adjustLauncher1Velocity(delta: Double) = adjustNectarLauncherVelocity(delta)
    fun adjustLauncher2Velocity(delta: Double) = adjustPollenLauncherVelocity(delta)

    /**
     * Updates intake and launcher controls:
     * - [spinUpLaunchers] (Button A): Spins Nectar & Pollen launchers up to target velocity
     * - [nectarTrigger] (Right Trigger): Fires Nectar launcher (Right Launcher) servo for 200ms upon trigger press
     * - [pollenTrigger] (Left Trigger): Fires Pollen launcher (Left Launcher) servo for 200ms upon trigger press
     * - [rightBumper]: Intake forward
     * - [leftBumper]: Reverse intake and launcher feed servos (unjam)
     */
    fun update(
        spinUpLaunchers: Boolean,
        nectarTrigger: Float,
        pollenTrigger: Float,
        rightBumper: Boolean,
        leftBumper: Boolean
    ) {
        // 1. Launchers Spin Up strictly via Button A
        if (spinUpLaunchers) {
            robot.nectarLauncher?.velocity = NECTAR_TARGET_VELOCITY
            robot.pollenLauncher?.velocity = POLLEN_TARGET_VELOCITY
        } else {
            robot.nectarLauncher?.velocity = 0.0
            robot.pollenLauncher?.velocity = 0.0
        }

        val currentNectarVel = robot.nectarLauncher?.velocity ?: 0.0
        val currentPollenVel = robot.pollenLauncher?.velocity ?: 0.0

        val nectarAtSpeed = currentNectarVel >= NECTAR_MIN_VELOCITY
        val pollenAtSpeed = currentPollenVel >= POLLEN_MIN_VELOCITY

        // 2. Intake Motor Control (Right Bumper = Forward, Left Bumper = Reverse)
        intakePower = when {
            rightBumper -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }
        robot.intake?.power = intakePower

        val currentTimeMs = System.currentTimeMillis()

        // 3. Nectar Launcher (Right) Trigger Edge Detection & 200ms Pulse
        if (nectarTrigger <= 0.05f) {
            nectarTriggerReleased = true
        } else if (nectarTrigger > 0.1f) {
            if (nectarAtSpeed && nectarTriggerReleased) {
                nectarTriggerReleased = false
                nectarServoEndTimeMs = currentTimeMs + SERVO_PULSE_DURATION_MS
            }
        }

        // 4. Pollen Launcher (Left) Trigger Edge Detection & 200ms Pulse
        if (pollenTrigger <= 0.05f) {
            pollenTriggerReleased = true
        } else if (pollenTrigger > 0.1f) {
            if (pollenAtSpeed && pollenTriggerReleased) {
                pollenTriggerReleased = false
                pollenServoEndTimeMs = currentTimeMs + SERVO_PULSE_DURATION_MS
            }
        }

        val nectarPulsing = currentTimeMs < nectarServoEndTimeMs
        val pollenPulsing = currentTimeMs < pollenServoEndTimeMs

        // 5. Calculate Servo Powers
        val nectarServoPower = when {
            nectarPulsing -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }

        val pollenServoPower = when {
            pollenPulsing -> 1.0
            leftBumper -> -1.0
            else -> 0.0
        }

        // Apply powers to Nectar and Pollen launcher feeder servos
        robot.nectarLauncherServo?.power = nectarServoPower
        robot.pollenLauncherServo?.power = pollenServoPower
    }

    override fun stop() {
        intakePower = 0.0
        robot.intake?.power = 0.0
        robot.nectarLauncherServo?.power = 0.0
        robot.pollenLauncherServo?.power = 0.0
        robot.nectarLauncher?.velocity = 0.0
        robot.pollenLauncher?.velocity = 0.0
        nectarTriggerReleased = true
        pollenTriggerReleased = true
        nectarServoEndTimeMs = 0L
        pollenServoEndTimeMs = 0L
    }
}
