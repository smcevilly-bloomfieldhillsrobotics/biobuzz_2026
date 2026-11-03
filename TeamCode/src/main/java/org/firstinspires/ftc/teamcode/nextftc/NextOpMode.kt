package org.firstinspires.ftc.teamcode.nextftc

import com.qualcomm.robotcore.eventloop.opmode.OpMode

/**
 * NextFTC Base OpMode (https://nextftc.dev/).
 * Manages background subsystems, event-driven gamepads, and command execution automatically.
 */
abstract class NextOpMode : OpMode() {

    companion object {
        private val registeredSubsystems = mutableListOf<NextSubsystem>()

        fun registerSubsystem(subsystem: NextSubsystem) {
            if (!registeredSubsystems.contains(subsystem)) {
                registeredSubsystems.add(subsystem)
            }
        }

        fun clearSubsystems() {
            registeredSubsystems.clear()
        }
    }

    protected lateinit var driver1: NextGamepad
    protected lateinit var driver2: NextGamepad

    private var activeCommand: NextCommand? = null

    override fun init() {
        clearSubsystems()
        driver1 = NextGamepad(gamepad1)
        driver2 = NextGamepad(gamepad2)
        onInit()
    }

    override fun start() {
        activeCommand = onStartCommand()
        activeCommand?.start()
    }

    override fun loop() {
        driver1.update()
        driver2.update()

        // Tick active commands
        activeCommand?.let { cmd ->
            cmd.execute()
            if (cmd.isFinished()) {
                cmd.end(false)
                activeCommand = null
            }
        }

        // Tick all registered subsystems
        registeredSubsystems.forEach { it.periodic() }

        onLoop()
    }

    override fun stop() {
        activeCommand?.end(true)
        registeredSubsystems.forEach { it.stop() }
        clearSubsystems()
    }

    open fun onInit() {}
    open fun onStartCommand(): NextCommand? = null
    open fun onLoop() {}
}
