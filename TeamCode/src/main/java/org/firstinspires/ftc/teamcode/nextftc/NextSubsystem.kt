package org.firstinspires.ftc.teamcode.nextftc

/**
 * NextFTC Subsystem Base Class (https://nextftc.dev/).
 * Encapsulates hardware components and registers background update tasks with NextOpMode.
 */
abstract class NextSubsystem {
    init {
        NextOpMode.registerSubsystem(this)
    }

    /**
     * Called automatically on every loop tick by NextOpMode.
     */
    open fun periodic() {}

    /**
     * Stops subsystem actuators when OpMode shuts down.
     */
    open fun stop() {}
}
