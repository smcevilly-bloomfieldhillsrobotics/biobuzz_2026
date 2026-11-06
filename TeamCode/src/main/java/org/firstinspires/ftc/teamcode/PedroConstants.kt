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
    // Robot is an 18" square. Odometry wheels are on the exterior (~8.0" - 8.5" lateral, or 7.5"-8.5" from center)
    // Both pods are symmetrical 1.5 inches back from center (-1.5" = -38.1 mm)
    // In goBILDA Pinpoint conventions:
    // X Pod Offset = lateral distance perpendicular to forward direction (+left / -right) = +8.0 inches (+203.2 mm)
    // Y Pod Offset = longitudinal distance perpendicular to strafe direction (+front / -rear) = -1.5 inches (-38.1 mm)
    @JvmField var X_POD_OFFSET_INCHES = 8.0
    @JvmField var Y_POD_OFFSET_INCHES = -1.5
    @JvmField var X_POD_OFFSET_MM = 203.2 // +203.2 mm (8.0" left of center)
    @JvmField var Y_POD_OFFSET_MM = -38.1 // -38.1 mm (1.5" back from center)

    // --- Path Follower PIDF Feedback Gains ---
    @JvmField var TRANSLATIONAL_P = 0.065
    @JvmField var TRANSLATIONAL_I = 0.000
    @JvmField var TRANSLATIONAL_D = 0.005

    @JvmField var HEADING_P = 0.700
    @JvmField var HEADING_I = 0.000
    @JvmField var HEADING_D = 0.040

    // Autonomous driving power limit (increased for faster, dynamic movement)
    @JvmField var MAX_AUTO_POWER = 0.80
}
