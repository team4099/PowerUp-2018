package org.usfirst.frc.team4099

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.serialization.json.JSON
import main.java.org.usfirst.frc.team4099.lib.util.AutoModeCreator
import org.usfirst.frc.team4099.auto.modes.AutoModeBase
import org.usfirst.frc.team4099.auto.modes.StandStillMode
import org.usfirst.frc.team4099.robot.Constants.Autonomous.AUTO_OPTIONS_DASHBOARD_KEY
import org.usfirst.frc.team4099.robot.Constants.Autonomous.SELECTED_AUTO_MODE_DASHBOARD_KEY


/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
object AutonomousSelector {
    private val defaultMode = AutoModeCreator("Stand Still", { StandStillMode() })
    private val allModes = arrayOf(
            defaultMode,
            AutoModeCreator("Move Forward", { StandStillMode() })
    )

    fun initAutoModeSelector() {
        var autoModesString = JSON.Companion.stringify(allModes)
        SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, autoModesString);
        SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, defaultMode.dashboardName);
    }

    fun getSelectedAutoMode(): AutoModeBase {
        var selectedModeName = SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, "no selected mode...")

        allModes.filter { it.dashboardName == selectedModeName }.map { return it.creator() }
        DriverStation.reportError("Failed to select a proper autonomous: $selectedModeName", false)
        return defaultMode.creator()
    }


}