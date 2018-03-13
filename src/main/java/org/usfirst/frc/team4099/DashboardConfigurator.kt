package org.usfirst.frc.team4099

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import main.java.org.usfirst.frc.team4099.lib.util.AutoModeCreator
import org.usfirst.frc.team4099.auto.modes.*
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
    private val defaultDelay = 0.0
    private val defaultStart = StartingPosition.CENTER
    private val defaultMode = AutoModeCreator("Line Cross", { startingPos, startingConfig, delay -> LineCrossMode(startingPos, startingConfig, delay) })

    enum class StartingPosition(val dashboardName: String)  {
        LEFT("LEFT"),
        CENTER("CENTER"),
        RIGHT("RIGHT")
    }
    private val allModes = arrayOf(
            defaultMode,
            AutoModeCreator("Stand Still", { _, _ ,_ -> StandStillMode() }),
            AutoModeCreator("Single Cube Switch", { startingPos, startingConfig, delay -> SingleCubeSwitch(startingPos, startingConfig, delay) }),
            AutoModeCreator("Single Cube Scale", { startingPos, startingConfig, delay -> SingleCubeScale(startingPos, startingConfig, delay) })
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
        for (i in 0 until StartingPosition.values().size - 1) {
            autoStartsString += "${StartingPosition.values()[i].printableName}, "
        }
        autoStartsString += "${StartingPosition.values()[StartingPosition.values().size - 1].printableName} ]"

        SmartDashboard.putString(AUTO_STARTS_DASHBOARD_KEY, autoStartsString)
        SmartDashboard.putString(SELECTED_AUTO_START_POS_KEY, defaultStart.printableName)

        SmartDashboard.putNumber(SELECTED_AUTO_START_DELAY_KEY, defaultDelay)
    }

    fun getSelectedAutoMode(allianceOwnership: String): AutoModeBase {
        val selectedModeName = SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, defaultMode.dashboardName)
        val selectedStartingPosition = SmartDashboard.getString(SELECTED_AUTO_START_POS_KEY, defaultStart.printableName)
        val selectedStartingDelay = SmartDashboard.getNumber(SELECTED_AUTO_START_DELAY_KEY, defaultDelay)

        val selectedStartEnum = StartingPosition.values().filter { it.printableName == selectedStartingPosition }[0]
        allModes.filter { it.dashboardName == selectedModeName }.map { return it.creator(selectedStartEnum, allianceOwnership, selectedStartingDelay) }
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