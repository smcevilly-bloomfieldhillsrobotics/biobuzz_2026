package org.firstinspires.ftc.teamcode

import com.bylazar.panels.PanelsConfig

/**
 * Custom configuration for Panels Dashboard (http://192.168.43.1:8001).
 * Panels automatically discovers and applies this configuration on startup.
 */
class CustomPanelsConfig : PanelsConfig() {
    override var isDisabled: Boolean = false
    override var enableLogs: Boolean = true
    override var enableClassCallerLogs: Boolean = false
}
