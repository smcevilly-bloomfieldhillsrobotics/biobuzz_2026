package org.firstinspires.ftc.teamcode.nextftc

import com.pedropathing.follower.Follower
import com.pedropathing.paths.Path

/**
 * NextFTC Pedro Pathing Follow Path Command (https://nextftc.dev/).
 * Natively wraps Pedro Pathing 3.0 Follower path execution into a NextCommand.
 */
class NextPedroFollowPath(
    private val follower: Follower,
    private val path: Path,
    private val timeoutMs: Long = 6000L
) : NextCommand {

    private var startTimeMs = 0L

    override fun start() {
        startTimeMs = System.currentTimeMillis()
        follower.follow(path)
        follower.update()
    }

    override fun execute() {
        follower.update()
    }

    override fun isFinished(): Boolean = !follower.isBusy() || (System.currentTimeMillis() - startTimeMs >= timeoutMs)

    override fun end(interrupted: Boolean) {
        if (interrupted || !follower.isBusy()) {
            follower.stop()
        }
    }
}

fun followPath(follower: Follower, path: Path, timeoutMs: Long = 6000L): NextCommand = NextPedroFollowPath(follower, path, timeoutMs)
