package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

/**
 * Autonomous OpMode that checks for a Red AprilTag (IDs 31-37).
 */
@Autonomous(name = "AprilTag Autonomous - Red (31-37)", group = "Autonomous")
class AprilTagRedAutoOpMode : AprilTagBaseAutoOpMode() {
    override val validTagIds: IntRange = 31..37
}