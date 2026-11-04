package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.nextftc.NextCommand

class LaunchRoutineCommand(
    private val robot: RobotHardware,
    private val durationMs: Long,
    private val tagValidator: (() -> Boolean)? = null,
    private val onStart: () -> Unit
) : NextCommand {

    private var startTimeMs = 0L
    private var phase = 0
    private var tagSeenInSpinup = false

    override fun start() {
        startTimeMs = System.currentTimeMillis()
        phase = 0
        tagSeenInSpinup = (tagValidator == null)
        onStart()
        
        robot.nectarLauncher?.velocity = IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY
        robot.pollenLauncher?.velocity = IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY
    }

    override fun execute() {
        val elapsed = System.currentTimeMillis() - startTimeMs

        // Continuously enforce target launcher motor velocities
        robot.nectarLauncher?.velocity = IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY
        robot.pollenLauncher?.velocity = IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY

        // Check if tag is seen within the window before servo activation
        if (tagValidator != null && tagValidator.invoke()) {
            tagSeenInSpinup = true
        }
        
        // After 1000ms spin-up, engage servos only if valid tag was confirmed
        if (phase == 0 && elapsed >= 1000L) {
            if (tagSeenInSpinup) {
                phase = 1
                robot.leftLauncherServo?.power = 1.0
                robot.rightLauncherServo?.power = 1.0
                robot.intake?.power = 0.8
            }
        }
    }

    override fun isFinished(): Boolean {
        return (System.currentTimeMillis() - startTimeMs) >= durationMs
    }

    override fun end(interrupted: Boolean) {
        robot.nectarLauncher?.velocity = 0.0
        robot.pollenLauncher?.velocity = 0.0
        robot.intake?.power = 0.0
        robot.leftLauncherServo?.power = 0.0
        robot.rightLauncherServo?.power = 0.0
    }
}
