package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection

/**
 * Pedro Pathing 3.0 Centralized Tuning Constants for Mecanum Drivetrain & Pinpoint Odometry.
 * Tunable live via FTC Dashboard (http://192.168.43.1:8080/dash).
 */
@Config
object PedroConstants {

    // --- Mecanum Drivetrain Physical Configuration ---
    @JvmField var TRACK_WIDTH_INCHES = 15.0
    @JvmField var WHEEL_BASE_INCHES = 14.0
    @JvmField var MAX_FORWARD_VELOCITY = 75.0 // in/sec (measured from Forward Tuner)
    @JvmField var MAX_STRAFE_VELOCITY = 65.0 // in/sec (measured from Lateral Tuner)
    @JvmField var MAX_ANGULAR_VELOCITY = 3.14 // rad/sec (measured from Turn Tuner)

    // --- Pinpoint Odometry Configuration ---
    @JvmField var PINPOINT_DEVICE_NAME = "odometry"
    
    // Encoder Pod Directions
    @JvmField var X_POD_DIRECTION = EncoderDirection.REVERSED
    @JvmField var Y_POD_DIRECTION = EncoderDirection.FORWARD

    // Custom Odometry Encoder Resolution / Ticks Per Unit
    // Default goBILDA 4-Bar Pod resolution is ~336.878 ticks/inch (13.2629 ticks/mm)
    @JvmField var USE_CUSTOM_TICKS_PER_INCH = false
    @JvmField var CUSTOM_TICKS_PER_INCH = 336.878

    // Pinpoint Odometry Pod Offsets (Measured from physical robot layout)
    // X Pod (Forward Pod): Far Left (+7.5 inches = +190.5 mm)
    // Y Pod (Strafe Pod): 1 3/4" back (-1.75 inches = -44.45 mm)
    @JvmField var X_POD_OFFSET_INCHES = 7.5
    @JvmField var Y_POD_OFFSET_INCHES = -1.75
    @JvmField var X_POD_OFFSET_MM = 190.5 // +190.5 mm left of center
    @JvmField var Y_POD_OFFSET_MM = -44.45 // -44.45 mm (1 3/4" back from center)

    // --- Path Follower PIDF Feedback Gains ---
    @JvmField var TRANSLATIONAL_P = 0.065
    @JvmField var TRANSLATIONAL_I = 0.000
    @JvmField var TRANSLATIONAL_D = 0.005

    @JvmField var HEADING_P = 0.700
    @JvmField var HEADING_I = 0.000
    @JvmField var HEADING_D = 0.040

    @JvmField var MAX_AUTO_POWER = 0.65
}
