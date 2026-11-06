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

        @JvmField var SERVO_PULSE_DURATION_MS = 200L // 200 milliseconds (0.2s)
        
        // Right (Nectar) Servo Positions: Rest at 0.5, Launch at 0.0
        @JvmField var RIGHT_SERVO_REST_POSITION = 0.5
        @JvmField var RIGHT_SERVO_LAUNCH_POSITION = 0.15

        // Left (Pollen) Servo Positions: Rest at 0.15, Launch at inverse of right servo (1.0 - 0.0 = 1.0)
        @JvmField var LEFT_SERVO_REST_POSITION = 0.15
        @JvmField val LEFT_SERVO_LAUNCH_POSITION = 0.5

        // Backward compatibility
        @JvmField var SERVO_REST_POSITION = 0.5
        @JvmField var SERVO_LAUNCH_POSITION = 0.0

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

    init {
        // Enforce rest positions immediately on init: Right = 0.5, Left = 0.15
        robot.nectarLauncherServo?.position = RIGHT_SERVO_REST_POSITION
        robot.pollenLauncherServo?.position = LEFT_SERVO_REST_POSITION
    }

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

    fun adjustPulseDuration(deltaMs: Long) {
        SERVO_PULSE_DURATION_MS = (SERVO_PULSE_DURATION_MS + deltaMs).coerceAtLeast(5L)
    }

    fun adjustLaunchPosition(delta: Double) {
        RIGHT_SERVO_LAUNCH_POSITION = (RIGHT_SERVO_LAUNCH_POSITION + delta).coerceIn(0.0, 1.0)
        SERVO_LAUNCH_POSITION = RIGHT_SERVO_LAUNCH_POSITION
    }

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

        // 3. Direct trigger hold detection (Hold trigger -> Launch position, Release trigger -> Rest position)
        // Servo moves only once the respective flywheel is up to speed (at or above MIN_VELOCITY)
        val nectarHeld = nectarTrigger > 0.1f && nectarAtSpeed
        val pollenHeld = pollenTrigger > 0.1f && pollenAtSpeed

        // 4. Calculate Servo Positions
        // Right (Nectar): Pressed & at speed -> RIGHT_SERVO_LAUNCH_POSITION, Released/below speed -> RIGHT_SERVO_REST_POSITION
        // Left (Pollen): Pressed & at speed -> LEFT_SERVO_LAUNCH_POSITION, Released/below speed -> LEFT_SERVO_REST_POSITION
        val nectarServoPos = if (nectarHeld) RIGHT_SERVO_LAUNCH_POSITION else RIGHT_SERVO_REST_POSITION
        val pollenServoPos = if (pollenHeld) LEFT_SERVO_LAUNCH_POSITION else LEFT_SERVO_REST_POSITION

        // Apply positions to Nectar and Pollen launcher feeder servos
        robot.nectarLauncherServo?.position = nectarServoPos
        robot.pollenLauncherServo?.position = pollenServoPos
    }

    override fun stop() {
        intakePower = 0.0
        robot.intake?.power = 0.0
        robot.nectarLauncherServo?.position = RIGHT_SERVO_REST_POSITION
        robot.pollenLauncherServo?.position = LEFT_SERVO_REST_POSITION
        robot.nectarLauncher?.velocity = 0.0
        robot.pollenLauncher?.velocity = 0.0
        nectarTriggerReleased = true
        pollenTriggerReleased = true
        nectarServoEndTimeMs = 0L
        pollenServoEndTimeMs = 0L
    }
}
