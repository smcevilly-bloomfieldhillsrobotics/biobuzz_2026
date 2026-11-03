package org.firstinspires.ftc.teamcode.nextftc

/**
 * NextFTC Command Interface & Builders (https://nextftc.dev/).
 */
interface NextCommand {
    fun start() {}
    fun execute() {}
    fun isFinished(): Boolean = true
    fun end(interrupted: Boolean) {}
}

class InstantCommand(private val action: () -> Unit) : NextCommand {
    override fun start() {
        action()
    }
    override fun isFinished(): Boolean = true
}

class DelayCommand(private val durationMs: Long) : NextCommand {
    private var startTimeMs = 0L

    override fun start() {
        startTimeMs = System.currentTimeMillis()
    }

    override fun isFinished(): Boolean =
        (System.currentTimeMillis() - startTimeMs) >= durationMs
}

class SeriesCommand(private val commands: List<NextCommand>) : NextCommand {
    private var currentIndex = 0
    private var currentStarted = false

    override fun start() {
        currentIndex = 0
        currentStarted = false
    }

    override fun execute() {
        if (currentIndex >= commands.size) return
        val cmd = commands[currentIndex]
        if (!currentStarted) {
            cmd.start()
            currentStarted = true
        }
        cmd.execute()
        if (cmd.isFinished()) {
            cmd.end(false)
            currentIndex++
            currentStarted = false
        }
    }

    override fun isFinished(): Boolean = currentIndex >= commands.size

    override fun end(interrupted: Boolean) {
        if (interrupted && currentIndex < commands.size) {
            commands[currentIndex].end(true)
        }
    }
}

class ParallelCommand(private val commands: List<NextCommand>) : NextCommand {
    private val finished = BooleanArray(commands.size)

    override fun start() {
        finished.fill(false)
        commands.forEach { it.start() }
    }

    override fun execute() {
        for (i in commands.indices) {
            if (!finished[i]) {
                commands[i].execute()
                if (commands[i].isFinished()) {
                    commands[i].end(false)
                    finished[i] = true
                }
            }
        }
    }

    override fun isFinished(): Boolean = finished.all { it }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            for (i in commands.indices) {
                if (!finished[i]) {
                    commands[i].end(true)
                }
            }
        }
    }
}

fun series(vararg commands: NextCommand): NextCommand = SeriesCommand(commands.toList())
fun parallel(vararg commands: NextCommand): NextCommand = ParallelCommand(commands.toList())
fun delay(durationMs: Long): NextCommand = DelayCommand(durationMs)
fun instant(action: () -> Unit): NextCommand = InstantCommand(action)
