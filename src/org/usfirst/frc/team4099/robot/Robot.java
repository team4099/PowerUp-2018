package org.usfirst.frc.team4099.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.SmartDashboardInteractions;
import org.usfirst.frc.team4099.auto.AutoModeExecuter;
import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.lib.util.CrashTracker;
import org.usfirst.frc.team4099.robot.drive.CDriveHelper;
import org.usfirst.frc.team4099.robot.drive.TankDriveHelper;
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender;
import org.usfirst.frc.team4099.robot.loops.Looper;
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator;
import org.usfirst.frc.team4099.robot.subsystems.Climber;
import org.usfirst.frc.team4099.robot.subsystems.Drive;
import org.usfirst.frc.team4099.robot.subsystems.Intake;

public class Robot extends IterativeRobot {

    private Drive mDrive = Drive.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Climber mClimber = Climber.getInstance();

    private CDriveHelper mCDriveHelper = CDriveHelper.getInstance();
    private TankDriveHelper mTDriveHelper = TankDriveHelper.getInstance();

    private ControlBoard mControls = ControlBoard.getInstance();
    private Looper mDisabledLooper = new Looper("disabledLooper");
    private Looper mEnabledLooper = new Looper("enabledLooper");

    private AutoModeExecuter mAutoModeExecuter = null;
    private SmartDashboardInteractions mSmartDashboardInteractions = new SmartDashboardInteractions();

    private boolean logging = true;
    private boolean isTurning = true;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }


    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            //TODO: add the robot state estimator here
            mEnabledLooper.register(mDrive.getLoop());
            mEnabledLooper.register(mIntake.getLoop());
            mEnabledLooper.register(mClimber.getLoop());
            mEnabledLooper.register(BrownoutDefender.getInstance());

            mDisabledLooper.register(VoltageEstimator.getInstance());

            mSmartDashboardInteractions.initWithDefaults();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("robotInit", t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            mEnabledLooper.stop(); // end EnabledLooper
            mDisabledLooper.start(); // start DisabledLooper

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("disabledInit", t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mDisabledLooper.stop(); // end DisabledLooper
            mEnabledLooper.start(); // start EnabledLooper
            mDrive.zeroSensors();

            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(mSmartDashboardInteractions.getSelectedAutonMode());
            mAutoModeExecuter.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("autonomousInit", t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            isTurning = true;
            mEnabledLooper.start(); // start EnabledLooper
            mDisabledLooper.stop(); // end DisabledLooper

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("teleopInit", t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            outputAllToSmartDashboard();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("disabledPeriodic", t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        try {
//            mDrive.turnAngle();
            outputAllToSmartDashboard();
            updateDashboardFeedback();
        } catch (Throwable t) {

            CrashTracker.logThrowableCrash("autonomousPeriodic", t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        try {
            double throttle = mControls.getThrottle();
            double turn = mControls.getTurn();
            boolean isQuickTurn = mControls.getQuickTurn();

            boolean toggleGrab = mControls.getToggleIntakeGrab();
            boolean toggleUp = mControls.getToggleIntakeUp();

            boolean climbing = mControls.getClimber();

            SmartDashboard.putBoolean("isQuickTurn", isQuickTurn);
            SmartDashboard.putNumber("voltage", VoltageEstimator.getInstance().getAverageVoltage());

            mDrive.setOpenLoop(mCDriveHelper.curvatureDrive(throttle, turn, isQuickTurn));

            //mDrive.setOpenLoop(mTDriveHelper.tankDrive(throttle, turn));
            mIntake.updateIntake(toggleUp, toggleGrab);
            if(climbing) {
                mClimber.setClimbingMode(Climber.ClimberState.CLIMBING);
            } else {
                mClimber.setClimbingMode(Climber.ClimberState.NOT_CLIMBING);
            }

            outputAllToSmartDashboard();
            updateDashboardFeedback(); // things such as is aligned?, etc

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("teleopPeriodic", t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logAutoInit();
            mEnabledLooper.start(); // start EnabledLooper
            mDisabledLooper.stop(); // end DisabledLooper
            mDrive.zeroSensors();
            mDrive.setAutonomousTurning();
            mDrive.setAngleSetpoint(90);
            isTurning = true;
            LiveWindow.setEnabled(true);
            startLiveWindowMode();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("testInit", t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        try {
            LiveWindow.run();
            if(isTurning) {
                isTurning = !mDrive.turnAngle();
            } else {
                mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            }
            System.out.println("isTurning:" + isTurning);
//            DriveSignal lSignal = new DriveSignal(1, -1);
//            DriveSignal rSignal = new DriveSignal(-1, 1);
//            if(mControls.getClimber()) {
//                mDrive.setOpenLoop(lSignal);
//            } else {
//                mDrive.setOpenLoop(rSignal);
//            }

            outputAllToSmartDashboard();
            updateLiveWindowTables();
            updateDashboardFeedback();
        } catch (Throwable t) {

            CrashTracker.logThrowableCrash("testPeriodic", t);
            throw t;
        }
    }

    /**
     * Log information from all subsystems onto the SmartDashboard
     */
    private void outputAllToSmartDashboard() {
        if (logging) {
            mDrive.outputToSmartDashboard(); // subsystems output to SmartDashboard
            mIntake.outputToSmartDashboard();
            mClimber.outputToSmartDashboard();
        }
    }

    private void startLiveWindowMode() {
        mDrive.startLiveWindowMode();
    }

    private void updateLiveWindowTables() {
        mDrive.updateLiveWindowTables();
    }

    private void updateDashboardFeedback() {
        // update things such as "is robot aligned with peg"
    }
}
