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
    private val defaultMode = AutoModeCreator("Switch", { startingPos, startingConfig, delay -> SingleCubeSwitch(startingPos, startingConfig, delay) })

    enum class StartingPosition(val dashboardName: String)  {
        LEFT("LEFT"),
        CENTER("CENTER"),
        RIGHT("RIGHT")
    }
    private val allModes = arrayOf(
            defaultMode,
            AutoModeCreator("Two Cube Switch", {startingPos, startingConfig, delay -> TwoCubeSwitch(startingPos, startingConfig, delay) }),
            AutoModeCreator("Edge Switch", {startingPos, startingConfig, delay -> SingleCubeEdgeSwitch(startingPos, startingConfig, delay)}),
            AutoModeCreator("Stand Still", { _, _ ,_ -> StandStillMode() }),
            AutoModeCreator("Line Cross", { startingPos, startingConfig, delay -> LineCrossMode(startingPos, startingConfig, delay) }),
            AutoModeCreator("Scale", { startingPos, startingConfig, delay -> SingleCubeScale(startingPos, startingConfig, delay) })
    )


    fun initDashboard() {
        // Give alliance color
        var color = ""
        while (color == "")
            color = DriverStation.getInstance().alliance.name
        SmartDashboard.putString(Constants.Dashboard.ALLIANCE_COLOR_KEY, color)

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
            autoStartsString += "${StartingPosition.values()[i].dashboardName}, "
        }
        autoStartsString += "${StartingPosition.values()[StartingPosition.values().size - 1].dashboardName} ]"

        SmartDashboard.putString(AUTO_STARTS_DASHBOARD_KEY, autoStartsString)
        SmartDashboard.putString(SELECTED_AUTO_START_POS_KEY, defaultStart.dashboardName)

        SmartDashboard.putNumber(SELECTED_AUTO_START_DELAY_KEY, defaultDelay)
    }

    fun getSelectedAutoMode(allianceOwnership: String): AutoModeBase {

        val selectedModeName = SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, defaultMode.dashboardName)
        val selectedStartingPosition = SmartDashboard.getString(SELECTED_AUTO_START_POS_KEY, defaultStart.dashboardName)
        val selectedStartingDelay = SmartDashboard.getNumber(SELECTED_AUTO_START_DELAY_KEY, defaultDelay)

        var selectedStartEnum = StartingPosition.RIGHT

        for (start in StartingPosition.values()) {
            if (start.dashboardName == selectedStartingPosition) {
                selectedStartEnum = start
                break
            }
        }

        for (mode in allModes) {
            if (mode.dashboardName == selectedModeName) {
                return mode.creator(selectedStartEnum, allianceOwnership, selectedStartingDelay)
            }
        }

        println("$selectedModeName $selectedStartingPosition")

        DriverStation.reportError("Failed to select a proper autonomous: $selectedModeName", false)
        return defaultMode.creator(defaultStart, allianceOwnership, selectedStartingDelay)
    }

    fun updateAllianceOwnership(): String {
        var allianceOwnership = ""
        // TODO: Check if this ever hangs
        while(allianceOwnership == "") {
            println("Waiting for alliance config...")
            allianceOwnership = DriverStation.getInstance().gameSpecificMessage
        }

        SmartDashboard.putString(Constants.Dashboard.ALLIANCE_OWNERSHIP_KEY, allianceOwnership)
        return allianceOwnership
    }


}