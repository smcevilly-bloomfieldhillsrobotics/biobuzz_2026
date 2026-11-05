package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.nextftc.NextCommand

class LaunchRoutineCommand(
    private val robot: RobotHardware,
    private val durationMs: Long,
    private val tagValidator: (() -> Boolean)? = null,
    private val onStatusUpdate: ((String) -> Unit)? = null,
    private val onStart: () -> Unit
) : NextCommand {

    private var startTimeMs = 0L
    private var phase = 0
    private var tagSeenInSpinup = false

    var isLaunching = false
        private set

    override fun start() {
        startTimeMs = System.currentTimeMillis()
        phase = 0
        tagSeenInSpinup = (tagValidator == null)
        onStart()
        
        robot.nectarLauncher?.velocity = IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY
        robot.pollenLauncher?.velocity = IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY
        robot.intake?.power = 1.0
    }

    override fun execute() {
        val elapsed = System.currentTimeMillis() - startTimeMs

        // Continuously enforce target launcher motor velocities
        robot.nectarLauncher?.velocity = IntakeLauncherSubsystem.NECTAR_TARGET_VELOCITY
        robot.pollenLauncher?.velocity = IntakeLauncherSubsystem.POLLEN_TARGET_VELOCITY

        // Continuously run intake the whole time the launch sequence is active
        robot.intake?.power = 1.0

        // Check if tag is seen within the window before servo activation
        if (tagValidator != null && tagValidator.invoke()) {
            tagSeenInSpinup = true
        }
        
        // Phase 0: Spin-up window (0 to 1000ms)
        if (phase == 0) {
            if (elapsed >= 1000L) {
                phase = 1
                if (tagSeenInSpinup) {
                    isLaunching = true
                    onStatusUpdate?.invoke("AprilTag confirmed! Launching elements.")
                } else {
                    isLaunching = false
                    onStatusUpdate?.invoke("AprilTag NOT found! Aborting launch.")
                }
            }
        }

        // Phase 1: Launch window (1000ms onwards)
        if (phase == 1) {
            if (isLaunching) {
                robot.leftLauncherServo?.position = IntakeLauncherSubsystem.SERVO_LAUNCH_POSITION
                robot.rightLauncherServo?.position = IntakeLauncherSubsystem.SERVO_LAUNCH_POSITION
            } else {
                robot.leftLauncherServo?.position = IntakeLauncherSubsystem.SERVO_REST_POSITION
                robot.rightLauncherServo?.position = IntakeLauncherSubsystem.SERVO_REST_POSITION
            }
        }
    }

    override fun isFinished(): Boolean {
        // If durationMs is <= 0 or Long.MAX_VALUE, run continuously for the remainder of autonomous
        if (durationMs <= 0 || durationMs == Long.MAX_VALUE) {
            return false
        }
        return (System.currentTimeMillis() - startTimeMs) >= durationMs
    }

    override fun end(interrupted: Boolean) {
        isLaunching = false
        robot.nectarLauncher?.velocity = 0.0
        robot.pollenLauncher?.velocity = 0.0
        robot.intake?.power = 0.0
        robot.leftLauncherServo?.position = IntakeLauncherSubsystem.SERVO_REST_POSITION
        robot.rightLauncherServo?.position = IntakeLauncherSubsystem.SERVO_REST_POSITION
    }
}
