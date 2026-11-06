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

        val currentSeg = pathTracker.current()
        val endPose = pathTracker.endPose()

        // Calculate a lookahead target point along the actual curve
        val robotPosVec = Vector2D.cartesian(robotPose.x(), robotPose.y())
        val targetPose: Pose = if (currentSeg != null) {
            val closestT = currentSeg.curve.closestParameter(robotPosVec)
            // Lookahead along the curve by ~0.15 of parameter t (roughly 10-15 inches)
            val lookaheadT = (closestT + 0.15).coerceIn(0.0, 1.0)
            currentSeg.get(lookaheadT)
        } else {
            endPose
        }

        // Target error vectors based on the lookahead point on the curve
        val dx = targetPose.x() - robotPose.x()
        val dy = targetPose.y() - robotPose.y()
        var headingError = targetPose.heading() - robotPose.heading()
        while (headingError > Math.PI) headingError -= 2 * Math.PI
        while (headingError < -Math.PI) headingError += 2 * Math.PI

        val distToEnd = hypot(endPose.x() - robotPose.x(), endPose.y() - robotPose.y())
        var finalHeadingError = endPose.heading() - robotPose.heading()
        while (finalHeadingError > Math.PI) finalHeadingError -= 2 * Math.PI
        while (finalHeadingError < -Math.PI) finalHeadingError += 2 * Math.PI

        if (distToEnd < 2.0 && abs(Math.toDegrees(finalHeadingError)) < 5.0) {
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
            turnCmd.coerceIn(-0.55, 0.55)
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
