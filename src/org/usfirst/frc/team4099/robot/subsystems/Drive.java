package org.usfirst.frc.team4099.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.lib.util.Utils;
import org.usfirst.frc.team4099.robot.loops.Loop;
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator;

public class Drive implements Subsystem {

    private static Drive sInstance = new Drive();
    private final Talon leftTalonSR,
                        rightTalonSR;
    private final AHRS ahrs;
    private DriveControlState currentState;

    private double lastLeft = 0;
    private double lastRight = 0;

    public enum DriveControlState {
        OPEN_LOOP;
    }

    private Drive() {
        leftTalonSR = new Talon(0);
        rightTalonSR = new Talon(1);

        ahrs = new AHRS(SPI.Port.kMXP);
    }

    public static Drive getInstance() {
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
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void zeroSensors() {

    }

    private synchronized void setOpenLeftRightPower(double left, double right) {
        double ldiff = Utils.diff(left, lastLeft);
        double rdiff = Utils.diff(right, lastRight);

        double MAX_OUTPUT = VoltageEstimator.getInstance().getAverageVoltage() / 14.0;
        double DAMPENING_FRACTION = 0.8;

        double l_dampen = MAX_OUTPUT -
                          (DAMPENING_FRACTION * MAX_OUTPUT) * ldiff;
        double r_dampen = MAX_OUTPUT -
                          (DAMPENING_FRACTION * MAX_OUTPUT) * rdiff;

        left *= l_dampen;
        right *= r_dampen;

        leftTalonSR.set(-left);
        rightTalonSR.set(right);

        lastLeft = left;
        lastRight = right;
    }

    private synchronized void setLeftRightPower(double left, double right) {
        leftTalonSR.set(-left);
        rightTalonSR.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (currentState != DriveControlState.OPEN_LOOP) {
            currentState = DriveControlState.OPEN_LOOP;
        }
        setOpenLeftRightPower(signal.getLeftMotor(), signal.getRightMotor());
        //setLeftRightPower(signal.getLeftMotor(), signal.getRightMotor());
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
