package org.firstinspires.ftc.teamcode

import com.pedropathing.algorithm.Algorithm
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.paths.PathTracker
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Pedro Pathing closed-loop trajectory algorithm.
 * Kept in its own dedicated file so OpModes remain 100% clean and free of algorithm math.
 */
class PedroAlgorithm : Algorithm {
    private var isBusy = false
    private var currentPose = Pose.zero()

    override fun reset() {
        isBusy = true
    }

    override fun isBusy(): Boolean = isBusy

    override fun calculatePath(
        drivetrain: Drivetrain,
        pathTracker: PathTracker,
        motionState: MotionState,
        dt: Double
    ): DrivePowers {
        val robotPose = motionState.pose()
        currentPose = robotPose

        val endPose = pathTracker.endPose()
        val dx = endPose.x() - robotPose.x()
        val dy = endPose.y() - robotPose.y()
        var headingError = endPose.heading() - robotPose.heading()
        while (headingError > Math.PI) headingError -= 2 * Math.PI
        while (headingError < -Math.PI) headingError += 2 * Math.PI

        if (hypot(dx, dy) < 2.5 && abs(Math.toDegrees(headingError)) < 6.0) {
            isBusy = false
            return DrivePowers.zero()
        }

        val cosH = cos(robotPose.heading())
        val sinH = sin(robotPose.heading())

        val fwdCmd = (dx * cosH + dy * sinH) * PedroConstants.TRANSLATIONAL_P
        val strCmd = (-dx * sinH + dy * cosH) * PedroConstants.TRANSLATIONAL_P
        val turnCmd = headingError * PedroConstants.HEADING_P

        return DrivePowers(
            fwdCmd.coerceIn(-PedroConstants.MAX_AUTO_POWER, PedroConstants.MAX_AUTO_POWER),
            strCmd.coerceIn(-PedroConstants.MAX_AUTO_POWER, PedroConstants.MAX_AUTO_POWER),
            turnCmd.coerceIn(-0.4, 0.4)
        )
    }

    override fun calculateHold(
        drivetrain: Drivetrain,
        targetPose: Pose,
        motionState: MotionState,
        useHoldScaling: Boolean,
        dt: Double
    ): DrivePowers = DrivePowers.zero()

    override fun completion(): Double = 1.0
    override fun closestPose(): Pose = currentPose
    override fun closestTangent(): Vector2D = Vector2D.zero()
    override fun closestNormal(): Vector2D = Vector2D.zero()
    override fun curvature(): Double = 0.0
    override fun remainingDistance(): Double = 0.0
    override fun atParametricEnd(): Boolean = true
    override fun debug(): Map<String, Any> = emptyMap()
}
