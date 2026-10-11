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

/**
 * DrivetrainSubsystem encapsulates arcade drive kinematics and motor power calculations.
 */
class DrivetrainSubsystem(private val robot: RobotHardware) {

    var leftPower: Double = 0.0
        private set
    var rightPower: Double = 0.0
        private set

    fun arcadeDrive(forward: Double, rotate: Double) {
        leftPower = forward + rotate
        rightPower = forward - rotate

        robot.leftDrive?.let { it.power = leftPower }
        robot.rightDrive?.let { it.power = rightPower }
    }

    fun stop() {
        leftPower = 0.0
        rightPower = 0.0
        robot.leftDrive?.power = 0.0
        robot.rightDrive?.power = 0.0
    }
}
