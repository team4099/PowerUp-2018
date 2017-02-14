package org.usfirst.frc.team4099.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.robot.loops.Loop;

public class Drive implements Subsystem, PIDOutput {

    private static Drive sInstance = new Drive();
    private Talon leftTalonSR,
                  rightTalonSR;
    private AHRS ahrs;
    private DriveControlState currentState;

    public enum DriveControlState {
        OPEN_LOOP,
        AUTONOMOUS
    }

    public enum AutonomousState {
        START,
        FIRST_FORWARD,
        TURN_TO_GOAL,
        FORWARD_TO_LIFT,
        BACK_OUT_OF_LIFT,
        TURN_TO_BASELINE,
        TURN_BEFORE_SECOND_GEAR,
        TURN_FOR_SECOND_GEAR,
        FORWARD_FOR_SECOND_GEAR,
        TURN_AFTER_SECOND_GEAR,
        TURN_FOR_SECOND_LIFT,
        FORWARD_FOR_SECOND_LIFT,
        BACK_OUT_OF_SECOND_LIFT,
        FORWARD_TO_BASELINE,
        TURN_AROUND_AFTER_BASELINE,
        WAIT,
    }

    public enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum AutonomousConfiguration {
        BASELINE,
        ONE_GEAR,
        TWO_GEAR
    }

    private static final double kP = 0.03;
    private static final double kI = 0.00;
    private static final double kD = 0.00;
    private static final double kF = 0.00;

    private static final double kToleranceDegrees = 2f;

    private double rotationRate;

    private PIDController turnController;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private double distanceToDrive;
    private double startingAngle = 0;

    private AutonomousState autonomousState = AutonomousState.START;
    private final StartingPosition startingPosition = StartingPosition.CENTER;
    private final AutonomousConfiguration autonomousConfiguration = AutonomousConfiguration.ONE_GEAR;

    private Drive() {
        leftTalonSR = new Talon(0);
        rightTalonSR = new Talon(1);

        ahrs = new AHRS(SPI.Port.kMXP);

        leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180d, 180d);
        turnController.setOutputRange(-1d, 1d);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);

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

        SmartDashboard.putNumber("leftTalon", leftTalonSR.get());
        SmartDashboard.putNumber("rightTalon", rightTalonSR.get());
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
        leftTalonSR.set(-left);
        rightTalonSR.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (currentState != DriveControlState.OPEN_LOOP) {
            currentState = DriveControlState.OPEN_LOOP;
        }

        setLeftRightPower(signal.getLeftMotor(), signal.getRightMotor());
    }

    public synchronized void setAutonomous() {
        currentState = DriveControlState.AUTONOMOUS;
    }

    @Override
    public void pidWrite(double output) {
        rotationRate = output;
    }

    private void turnAngle() {
        leftTalonSR.set(rotationRate);
        rightTalonSR.set(-rotationRate);
    }

    public boolean turnAngle(double relativeAngle) {
        if (!turnController.isEnabled()) {
            turnController.enable();
            turnController.setSetpoint(Math.IEEEremainder(relativeAngle - startingAngle, 360) - 180);
            startingAngle = ahrs.getAngle();
            turnAngle();
            return true;
        } else if (Math.abs(ahrs.getAngle() - Math.IEEEremainder(relativeAngle - startingAngle, 360)) < kToleranceDegrees) {
            turnController.disable();
            leftTalonSR.set(0);
            rightTalonSR.set(0);
            return false;
        } else {
            turnAngle();
            return true;
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
