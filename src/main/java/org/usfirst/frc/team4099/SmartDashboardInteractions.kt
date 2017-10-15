package org.usfirst.frc.team4099

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.auto.AutoModeEndedException
import org.usfirst.frc.team4099.auto.modes.AutoModeBase
import org.usfirst.frc.team4099.auto.modes.BaselineMode
import org.usfirst.frc.team4099.auto.modes.OneGearMode
import org.usfirst.frc.team4099.auto.modes.StandStillMode
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters
import org.usfirst.frc.team4099.lib.util.Rotation2D
import java.util.*

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
class SmartDashboardInteractions {

    fun initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false)
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, true)

        val autoOptionsArray = ArrayList<String>(AutonOption.values().size)
        AutonOption.values().mapTo(autoOptionsArray) { it.nameStr }
        SmartDashboard.putString(AUTO_OPTIONS, autoOptionsArray.toString())
        SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name)
        SmartDashboard.putString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString)
    }

    val isInHoodTuningMode: Boolean
        get() = SmartDashboard.getBoolean(HOOD_TUNING_MODE, false)

    fun shouldLogToSmartDashboard(): Boolean {
        return SmartDashboard.getBoolean(OUTPUT_TO_SMART_DASHBOARD, true)
    }

    val selectedAutonMode: AutoModeBase
        get() {
            val autoModeString = SmartDashboard.getString(SELECTED_AUTO_MODE, DEFAULT_MODE.name)
            var selectedOption = DEFAULT_MODE
            for (autonOption in AutonOption.values()) {
                if (autonOption.nameStr == autoModeString) {
                    selectedOption = autonOption
                    break
                }
            }

            val autoLaneString = SmartDashboard.getString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString)
            var selectedLane = DEFAULT_LANE
            for (autonLane in AutonLane.values()) {
                if (autonLane.numberString == autoLaneString) {
                    selectedLane = autonLane
                }
            }

            return createAutoMode(selectedOption, selectedLane)
        }


    /**
     * I don't trust SendableChooser to manage [AutoModeBase]
     * objects directly, so use this enum to project us from WPILIb.
     */
    internal enum class AutonOption constructor(val nameStr: String) {
        ONE_GEAR("One Gear"),
        BASELINE("Baseline"),
        TEST_DRIVE("TEST ONLY Driving")
    }

    internal enum class AutonLane private constructor(val distanceToDrive: Double, val numberString: String) {
        LEFT_LANE(2.5, "1"), CENTER_LANE(6.0, "2"), RIGHT_LANE(2.25, "3")
    }

    private fun getAimingHintForLane(lane: AutonLane): AutonomousInitParameters {
        return when (lane) {
            AutonLane.LEFT_LANE -> AutonomousInitParameters(lane.distanceToDrive, Rotation2D.fromDegrees(60.0), -1)
            AutonLane.CENTER_LANE -> AutonomousInitParameters(lane.distanceToDrive, Rotation2D.fromDegrees(0.0), -1)
            else -> /* if (lane == AutonLane.RIGHT_LANE) {*/
                AutonomousInitParameters(lane.distanceToDrive, Rotation2D.fromDegrees(-60.0), -1)
        }
    }

    private fun createAutoMode(autonOption: AutonOption, autonLane: AutonLane): AutoModeBase {
        when (autonOption) {
            SmartDashboardInteractions.AutonOption.ONE_GEAR -> return OneGearMode(getAimingHintForLane(autonLane), false, false)
            SmartDashboardInteractions.AutonOption.BASELINE -> return BaselineMode(getAimingHintForLane(autonLane), false)
            SmartDashboardInteractions.AutonOption.TEST_DRIVE -> return object : AutoModeBase() {
                @Throws(AutoModeEndedException::class)
                override fun routine() {
                    throw RuntimeException("Expected exception!!!")
                }
            }
            else -> {
                println("ERROR: unexpected auto mode: " + autonOption)
                return StandStillMode()
            }
        }
    }

    companion object {

        private val HOOD_TUNING_MODE = "Hood Tuning Mode"
        private val OUTPUT_TO_SMART_DASHBOARD = "Output To SmartDashboard"

        private val AUTO_OPTIONS = "auto_options"
        private val SELECTED_AUTO_MODE = "selected_auto_mode"
        private val SELECTED_AUTO_LANE = "selected_auto_lane"

        private val DEFAULT_MODE = AutonOption.ONE_GEAR
        private val DEFAULT_LANE = AutonLane.CENTER_LANE
    }
}