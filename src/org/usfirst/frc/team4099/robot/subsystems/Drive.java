package org.usfirst.frc.team4099.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.loops.Loop;

public class Drive implements Subsystem {

    private static Drive sInstance = new Drive();
    private Talon leftFrontTalonSR,
                  leftBackTalonSR,
                  rightFrontTalonSR,
                  rightBackTalonSR;
    private AHRS ahrs;
    private DriveControlState currentState = DriveControlState.OPEN_LOOP;

    public enum DriveControlState {
        OPEN_LOOP;
    }

    private Drive() {
        leftFrontTalonSR = new Talon(Constants.Drive.LEFT_FRONT_ID);
        leftBackTalonSR = new Talon(Constants.Drive.LEFT_BACK_ID);

        rightFrontTalonSR = new Talon(Constants.Drive.RIGHT_FRONT_ID);
        rightBackTalonSR = new Talon(Constants.Drive.RIGHT_BACK_ID);

        ahrs = new AHRS(SPI.Port.kMXP);
    }

    public static Drive getInstance() { // singleton
        return sInstance;
    }

    public AHRS getAHRS() {
        if (ahrs.isConnected())
            return ahrs;
        return null;
    }

    @Override
    public void outputToSmartDashboard() {
        if (this.getAHRS() != null)
            SmartDashboard.putNumber("gyro", this.getAHRS().getAngle());
        else
            SmartDashboard.putNumber("gyro", -31337);

        SmartDashboard.putNumber("leftTalon", leftFrontTalonSR.get());
        SmartDashboard.putNumber("rightTalon", rightFrontTalonSR.get());
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void zeroSensors() {

    }

    /**
     * Powers the left and right talons during OPEN_LOOP
     * @param left
     * @param right
     */
    private synchronized void setLeftRightPower(double left, double right) {
        leftFrontTalonSR.set(-left);
        leftBackTalonSR.set(-left);
        rightFrontTalonSR.set(right);
        rightBackTalonSR.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (currentState != DriveControlState.OPEN_LOOP) {
            currentState = DriveControlState.OPEN_LOOP;
        }

        setLeftRightPower(signal.getLeftMotor(), signal.getRightMotor());
    }

    public Loop getLoop() {
        return mLoop;
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }

        @Override
        public void onLoop() {
            synchronized (Drive.this) {
                switch (currentState) {
                    case OPEN_LOOP:
                        return;
                }
            }
        }

        @Override
        public void onStop() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }
    };
}
