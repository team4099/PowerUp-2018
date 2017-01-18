package org.usfirst.frc.team4099.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.util.CrashTracker;
import org.usfirst.frc.team4099.robot.drive.CDriveHelper;
import org.usfirst.frc.team4099.robot.drive.TankDriveHelper;
import org.usfirst.frc.team4099.robot.loops.Looper;
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator;
import org.usfirst.frc.team4099.robot.subsystems.Drive;

public class Robot extends IterativeRobot {

    private Drive mDrive = Drive.getInstance();
    private CDriveHelper mCDriveHelper = CDriveHelper.getInstance();
    private TankDriveHelper mTDriveHelper = TankDriveHelper.getInstance();

    private ControlBoard mControls = ControlBoard.getInstance();
    private Looper mDisabledLooper = new Looper("disabledLooper");
    private Looper mEnabledLooper = new Looper("enabledLooper");

    private boolean logging = true;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            //TODO: add the robot state estimator here
            mEnabledLooper.register(mDrive.getLoop());

            mDisabledLooper.register(VoltageEstimator.getInstance());

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

            mEnabledLooper.start(); // start EnabledLooper
            mDisabledLooper.stop(); // end DisabledLooper

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("autonomousInit", t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

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

            SmartDashboard.putBoolean("isQuickTurn", isQuickTurn);
            SmartDashboard.putNumber("voltage", VoltageEstimator.getInstance().getAverageVoltage());

            mDrive.setOpenLoop(mCDriveHelper.curvatureDrive(throttle, turn, isQuickTurn));
            //mDrive.setOpenLoop(mTDriveHelper.tankDrive(throttle, turn));

            outputAllToSmartDashboard();
            updateDashboardFeedback(); // things such as is aligned?, etc

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("teleopPeriodic", t);
            throw t;
        }
    }

    /**
     * Log information from all subsystems onto the SmartDashboard
     */
    private void outputAllToSmartDashboard() {
        if (logging) {
            mDrive.outputToSmartDashboard(); // subsystems output to SmartDashboard
        }
    }

    private void updateDashboardFeedback() {
        // update things such as "is robot aligned with peg"
    }
}
