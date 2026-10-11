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
        @JvmField var TARGET_VELOCITY = 1300.0
        @JvmField var MIN_VELOCITY = 1250.0
    }

    var intakePower: Double = 0.0
        private set

    fun update(rightTrigger: Float, leftTrigger: Float, rightBumper: Boolean) {
        intakePower = (rightTrigger - leftTrigger).toDouble()

        robot.intake?.let { it.power = intakePower }
        robot.leftIntakeServo?.let { it.power = intakePower }
        robot.rightIntakeServo?.let { it.power = intakePower }

        val windmillPower = if (rightBumper) {
            robot.launcher?.velocity = TARGET_VELOCITY
            val currentVel = robot.launcher?.velocity ?: 0.0
            if (currentVel > MIN_VELOCITY) {
                intakePower = (intakePower + 0.5).coerceIn(-1.0, 1.0)
                robot.intake?.let { it.power = intakePower }
                1.0
            } else {
                0.0
            }
        } else {
            robot.launcher?.velocity = 0.0
            if (leftTrigger > rightTrigger) {
                -1.0
            } else {
                0.0
            }
        }

        robot.windmillServo?.let { it.power = windmillPower }
    }

    fun stop() {
        intakePower = 0.0
        robot.intake?.power = 0.0
        robot.leftIntakeServo?.power = 0.0
        robot.rightIntakeServo?.power = 0.0
        robot.launcher?.velocity = 0.0
        robot.windmillServo?.power = 0.0
    }
}
