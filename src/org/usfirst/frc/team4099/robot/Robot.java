package org.usfirst.frc.team4099.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import org.usfirst.frc.team4099.lib.util.CrashTracker;
import org.usfirst.frc.team4099.robot.drive.CDriveHelper;
import org.usfirst.frc.team4099.robot.drive.TankDriveHelper;
import org.usfirst.frc.team4099.robot.subsystems.Drive;

public class Robot extends IterativeRobot {

    private Drive mDrive = Drive.getInstance();
    private CDriveHelper mCDriveHelper = CDriveHelper.getInstance();
    private TankDriveHelper mTDriveHelper = TankDriveHelper.getInstance();
    private ControlBoard mControls = ControlBoard.getInstance();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("robotInit", t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("disabledInit", t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("autonomousInit", t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("teleopInit", t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {


        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("disabledPeriodic", t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        try {


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
            mDrive.setOpenLoop(mCDriveHelper.curvatureDrive(throttle, turn, isQuickTurn));

            //System.out.println(mDrive.getAHRS().getAngle());
            System.out.println(isQuickTurn);


        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("teleopPeriodic", t);
            throw t;
        }
    }
}
