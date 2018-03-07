package org.usfirst.frc.team4099

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import main.java.org.usfirst.frc.team4099.lib.util.AutoModeCreator
import org.usfirst.frc.team4099.auto.modes.AutoModeBase
import org.usfirst.frc.team4099.auto.modes.StandStillMode
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.Constants.Autonomous.AUTO_OPTIONS_DASHBOARD_KEY
import org.usfirst.frc.team4099.robot.Constants.Autonomous.AUTO_STARTS_DASHBOARD_KEY
import org.usfirst.frc.team4099.robot.Constants.Autonomous.SELECTED_AUTO_MODE_DASHBOARD_KEY
import org.usfirst.frc.team4099.robot.Constants.Autonomous.SELECTED_AUTO_START_DELAY_KEY
import org.usfirst.frc.team4099.robot.Constants.Autonomous.SELECTED_AUTO_START_POS_KEY


/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
object DashboardConfigurator {
    private val defaultStart = "CENTER"
    private val defaultDelay = 0.0
    private val defaultMode = AutoModeCreator("Stand Still", { _, _, _ -> StandStillMode() })

    private val startingPositions = arrayOf(
            defaultStart,
            "LEFT",
            "RIGHT"
    )
    private val allModes = arrayOf(
            defaultMode,
            AutoModeCreator("Move Forward", { _, _, _ -> StandStillMode() })
    )


    fun initDashboard() {
        // Give alliance color
        SmartDashboard.putString(Constants.Dashboard.ALLIANCE_COLOR_KEY, DriverStation.getInstance().alliance.name)

        // Set up autonomous selector
        var autoModesString = "[ "
        for (i in 0 until allModes.size - 1) {
            autoModesString += "${allModes[i].dashboardName}, "
        }
        autoModesString += "${allModes[allModes.size - 1].dashboardName} ]"

        SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, autoModesString);
        SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, defaultMode.dashboardName);

        var autoStartsString = "[ "
        for (i in 0 until startingPositions.size - 1) {
            autoStartsString += "${startingPositions[i]}, "
        }
        autoStartsString += "${startingPositions[allModes.size - 1]} ]"

        SmartDashboard.putString(AUTO_STARTS_DASHBOARD_KEY, autoStartsString)
        SmartDashboard.putString(SELECTED_AUTO_START_POS_KEY, defaultStart)

        SmartDashboard.putNumber(SELECTED_AUTO_START_DELAY_KEY, defaultDelay)
    }

    fun getSelectedAutoMode(allianceOwnership: String): AutoModeBase {
        val selectedModeName = SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, defaultMode.dashboardName)
        val selectedStartingPosition = SmartDashboard.getString(SELECTED_AUTO_START_POS_KEY, defaultStart)
        val selectedStartingDelay = SmartDashboard.getNumber(SELECTED_AUTO_START_DELAY_KEY, defaultDelay)

        allModes.filter { it.dashboardName == selectedModeName }.map { return it.creator(selectedStartingPosition, allianceOwnership, selectedStartingDelay) }
        DriverStation.reportError("Failed to select a proper autonomous: $selectedModeName", false)
        return defaultMode.creator(defaultStart, allianceOwnership, defaultDelay)
    }

    fun updateAllianceOwnership(): String {
        var allianceOwnership = ""
        // TODO: Check if this ever hangs
        while(allianceOwnership == "") {
            allianceOwnership = DriverStation.getInstance().gameSpecificMessage
        }

        SmartDashboard.putString(Constants.Dashboard.ALLIANCE_OWNERSHIP_KEY, allianceOwnership)
        return allianceOwnership
    }


}