package org.usfirst.frc.team4099.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.lib.drive.PIDOutputReceiver;
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
        OPEN_LOOP,
        AUTONOMOUS_TURNING,
        AUTONOMOUS_DRIVING
    }

    private static final double kP = 0.03;
    private static final double kI = 0.00;
    private static final double kD = 0.00;
    private static final double kF = 0.00;

    private static final double kToleranceDegrees = 2f;
    private static final double kToleranceDistance = .05f;

    private PIDController turnController;
    private PIDController leftController;
    private PIDController rightController;

    private PIDOutputReceiver turnReceiver;
    private PIDOutputReceiver leftReceiver;
    private PIDOutputReceiver rightReceiver;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private double distanceToDrive;
    private double startingAngle = 0;

    private Drive() {
        leftFrontTalonSR = new Talon(Constants.Drive.LEFT_FRONT_ID);
        leftBackTalonSR = new Talon(Constants.Drive.LEFT_BACK_ID);

        rightFrontTalonSR = new Talon(Constants.Drive.RIGHT_FRONT_ID);
        rightBackTalonSR = new Talon(Constants.Drive.RIGHT_BACK_ID);

        ahrs = new AHRS(SPI.Port.kMXP);

        leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);

        turnReceiver = new PIDOutputReceiver();
        turnController = new PIDController(kP, kI, kD, kF, ahrs, turnReceiver);
        turnController.setInputRange(-180d, 180d);
        turnController.setOutputRange(-1d, 1d);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);

//        leftReceiver = new PIDOutputReceiver();
//        leftController = new PIDController()

        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
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
        if (this.getAHRS() != null) {
            SmartDashboard.putNumber("gyro", this.getAHRS().getAngle());
        }
        else {
            SmartDashboard.putNumber("gyro", -31337);
        }

        SmartDashboard.putNumber("leftTalon", leftFrontTalonSR.get());
        SmartDashboard.putNumber("rightTalon", rightFrontTalonSR.get());
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void zeroSensors() {
        if(ahrs.isConnected()) {
            ahrs.reset();
        }
        leftEncoder.reset();
        rightEncoder.reset();
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

    public synchronized void setAutonomousTurning() {
        currentState = DriveControlState.AUTONOMOUS_TURNING;
    }

    public synchronized void setAutonomousDriving() {
        currentState = DriveControlState.AUTONOMOUS_DRIVING;
    }

    private void turnAngle() {
        setLeftRightPower(turnReceiver.getOutput(), -turnReceiver.getOutput());
    }

    public boolean turnAngle(double relativeAngle) {
        setAutonomousTurning();
        if(!turnController.isEnabled()) {
            turnController.enable();
            turnController.setSetpoint(Math.IEEEremainder(relativeAngle - startingAngle, 360) - 180);
            startingAngle = ahrs.getAngle();
            turnAngle();
            return false;
        } else if(Math.abs(ahrs.getAngle() - Math.IEEEremainder(relativeAngle - startingAngle, 360)) < kToleranceDegrees) {
            turnController.disable();
            setLeftRightPower(0, 0);
            setOpenLoop(DriveSignal.NEUTRAL);
            return true;
        } else {
            turnAngle();
            return false;
        }
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
                        break;
                }
            }
        }

        @Override
        public void onStop() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }
    };
}
