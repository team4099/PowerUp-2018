package org.usfirst.frc.team4099.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.lib.util.Utils;
import org.usfirst.frc.team4099.robot.loops.Loop;
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator;

public class Drive implements Subsystem {

    private static Drive sInstance = new Drive();
    private Talon leftTalonSR,
                  rightTalonSR;
    private AHRS ahrs;
    private DriveControlState currentState;

    public enum DriveControlState {
        OPEN_LOOP;
    }

    private Drive() {
        leftTalonSR = new Talon(0);
        rightTalonSR = new Talon(1);

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

        SmartDashboard.putNumber("leftTalon", leftTalonSR.get());
        SmartDashboard.putNumber("rightTalon", rightTalonSR.get());
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
        leftTalonSR.set(-left);
        rightTalonSR.set(right);
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
