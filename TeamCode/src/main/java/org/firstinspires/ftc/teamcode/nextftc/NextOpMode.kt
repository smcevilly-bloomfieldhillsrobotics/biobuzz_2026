package org.firstinspires.ftc.teamcode.nextftc

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode

/**
 * NextFTC Base OpMode (https://nextftc.dev/).
 * Manages background subsystems, event-driven gamepads, and command execution automatically.
 * Inherits from LinearOpMode instead of OpMode so that third-party tools like Panels or FTC Dashboard
 * natively detect it via reflection when scanning the package.
 */
abstract class NextOpMode : LinearOpMode() {

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

    override fun runOpMode() {
        clearSubsystems()
        driver1 = NextGamepad(gamepad1)
        driver2 = NextGamepad(gamepad2)
        onInit()

        // Loop while INIT
        while (opModeInInit()) {
            driver1.update()
            driver2.update()
            onLoop()
            sleep(20)
        }

        if (isStopRequested) return

        // Started
        activeCommand = onStartCommand()
        activeCommand?.start()

        // Loop while PLAYING
        while (opModeIsActive()) {
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
            sleep(10) // Small sleep to yield to hardware
        }

        // Stopped
        activeCommand?.end(true)
        registeredSubsystems.forEach { it.stop() }
        clearSubsystems()
    }

    open fun onInit() {}
    open fun onStartCommand(): NextCommand? = null
    open fun onLoop() {}
}
