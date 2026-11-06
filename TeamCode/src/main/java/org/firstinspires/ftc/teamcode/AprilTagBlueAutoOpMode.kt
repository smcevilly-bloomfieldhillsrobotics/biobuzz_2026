package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

/**
 * Autonomous OpMode that checks for a Blue AprilTag (IDs 38-45).
 */
@Autonomous(name = "AprilTag Autonomous - Blue (38-45)", group = "Autonomous")
class AprilTagBlueAutoOpMode : AprilTagBaseAutoOpMode() {
    override val isBlueAlliance: Boolean = true
    override val validTagIds: IntRange = 38..45
}