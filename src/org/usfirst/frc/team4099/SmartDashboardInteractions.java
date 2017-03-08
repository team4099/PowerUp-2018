package org.usfirst.frc.team4099;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.modes.*;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2D;

import java.util.ArrayList;

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions {

    private static final String HOOD_TUNING_MODE = "Hood Tuning Mode";
    private static final String OUTPUT_TO_SMART_DASHBOARD = "Output To SmartDashboard";

    private static final String AUTO_OPTIONS = "auto_options";
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    private static final String SELECTED_AUTO_LANE = "selected_auto_lane";

    private static final AutonOption DEFAULT_MODE = AutonOption.ONE_GEAR;
    private static final AutonLane DEFAULT_LANE = AutonLane.LEFT_LANE;

    public void initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false);
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, true);

        ArrayList<String> autoOptionsArray = new ArrayList<>(AutonOption.values().length);
        for (AutonOption autonOption : AutonOption.values()) {
            autoOptionsArray.add(autonOption.name);
        }
        SmartDashboard.putString(AUTO_OPTIONS, autoOptionsArray.toString());
        SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        SmartDashboard.putString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString);
    }

    public boolean isInHoodTuningMode() {
        return SmartDashboard.getBoolean(HOOD_TUNING_MODE, false);
    }

    public boolean shouldLogToSmartDashboard() {
        return SmartDashboard.getBoolean(OUTPUT_TO_SMART_DASHBOARD, true);
    }

    public AutoModeBase getSelectedAutonMode() {
        String autoModeString = SmartDashboard.getString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        AutonOption selectedOption = DEFAULT_MODE;
        for (AutonOption autonOption : AutonOption.values()) {
            if (autonOption.name.equals(autoModeString)) {
                selectedOption = autonOption;
                break;
            }
        }

        String autoLaneString = SmartDashboard.getString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString);
        AutonLane selectedLane = DEFAULT_LANE;
        for (AutonLane autonLane : AutonLane.values()) {
            if (autonLane.numberString.equals(autoLaneString)) {
                selectedLane = autonLane;
            }
        }

        return createAutoMode(selectedOption, selectedLane);
    }


    /**
     * I don't trust SendableChooser to manage {@link AutoModeBase}
     * objects directly, so use this enum to project us from WPILIb.
     */
    enum AutonOption {
        ONE_GEAR("One Gear"),
        ONE_GEAR_AND_BACK_OUT("One Gear, Back Out"),
        ONE_GEAR_AND_BACK_OUT_AND_TURN("One Gear, Back Out, Turn Around"),
        TWO_GEAR("Two Gears"),
        TWO_GEAR_AND_BACK_OUT("Two Gears, Back Out"),
        TWO_GEAR_AND_BACK_OUT_AND_TURN("Two Gears, Back Out, Turn Around"),
        BASELINE("Baseline"),
        BASELINE_AND_TURN("Baseline, Turn Around"),
        STAND_STILL("Stand Still"),
        TEST_DRIVE("TEST ONLY Driving");

        public final String name;

        AutonOption(String name) {
            this.name = name;
        }
    }

    enum AutonLane {
        LEFT_LANE(60, "1"), CENTER_LANE(60, "2"), RIGHT_LANE(75, "3");

        public final double distanceToDrive;
        public final String numberString;

        AutonLane(double distanceToDrive, String numberString) {
            this.distanceToDrive = distanceToDrive;
            this.numberString = numberString;
        }
    }

    private AutonomousInitParameters getAimingHintForLane(AutonLane lane) {
        if (lane == AutonLane.LEFT_LANE) {
            return new AutonomousInitParameters(lane.distanceToDrive, Rotation2D.fromDegrees(60), -1);
        } else if (lane == AutonLane.CENTER_LANE) {
            return new AutonomousInitParameters(lane.distanceToDrive, Rotation2D.fromDegrees(0), -1);
        } else {/* if (lane == AutonLane.RIGHT_LANE) {*/
            return new AutonomousInitParameters(lane.distanceToDrive, Rotation2D.fromDegrees(-60), -1);
        }
    }

    private AutoModeBase createAutoMode(AutonOption autonOption, AutonLane autonLane) {
        switch (autonOption) {
            case ONE_GEAR:
                return new OneGearMode(getAimingHintForLane(autonLane), false, false);
            case ONE_GEAR_AND_BACK_OUT:
                return new OneGearMode(getAimingHintForLane(autonLane), true, false);
            case ONE_GEAR_AND_BACK_OUT_AND_TURN:
                return new OneGearMode(getAimingHintForLane(autonLane), true, true);
            case TWO_GEAR:
                return new TwoGearMode(getAimingHintForLane(autonLane), false, false);
            case TWO_GEAR_AND_BACK_OUT:
                return new TwoGearMode(getAimingHintForLane(autonLane), true, false);
            case TWO_GEAR_AND_BACK_OUT_AND_TURN:
                return new TwoGearMode(getAimingHintForLane(autonLane), true, true);
            case BASELINE:
                return new BaselineMode(getAimingHintForLane(autonLane), false);
            case BASELINE_AND_TURN:
                return new BaselineMode(getAimingHintForLane(autonLane), true);
            case TEST_DRIVE:
                return new AutoModeBase() {
                    @Override
                    protected void routine() throws AutoModeEndedException {
                        throw new RuntimeException("Expected exception!!!");
                    }
                };

            case STAND_STILL: // fallthrough
            default:
                System.out.println("ERROR: unexpected auto mode: " + autonOption);
                return new StandStillMode();
        }
    }
}