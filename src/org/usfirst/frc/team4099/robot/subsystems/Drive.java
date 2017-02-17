package org.usfirst.frc.team4099.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.sun.tools.internal.jxc.ap.Const;
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

    private static final double kUTurn = 0.015;

    private static final double kPTurn = 0.015;
    private static final double kITurn = 0.00;
    private static final double kDTurn = 0.6;
    private static final double kFTurn = 0.00;

    private static final double kPForward = 0.015;
    private static final double kIForward = 0.00;
    private static final double kDForward = 0.6;
    private static final double kFForward = 0.00;

    private static final double kToleranceDistance = .05f;

    private PIDController turnController;
    private PIDController leftController;
    private PIDController rightController;

    private PIDOutputReceiver turnReceiver;
    private PIDOutputReceiver leftReceiver;
    private PIDOutputReceiver rightReceiver;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private double leftDistancePerPulse = 1;
    private double rightDistancePerPulse = 1;

    private double distanceToDrive;
    private double startingAngle = 0;

    private Drive() {
        leftFrontTalonSR = new Talon(Constants.Drive.LEFT_FRONT_ID);
        leftBackTalonSR = new Talon(Constants.Drive.LEFT_BACK_ID);

        rightFrontTalonSR = new Talon(Constants.Drive.RIGHT_FRONT_ID);
        rightBackTalonSR = new Talon(Constants.Drive.RIGHT_BACK_ID);

        ahrs = new AHRS(SPI.Port.kMXP);

        leftEncoder = new Encoder(Constants.Drive.LEFT_ENCODER_A, Constants.Drive.LEFT_ENCODER_B, false, Encoder.EncodingType.k4X);
        leftEncoder.setDistancePerPulse(Constants.Drive.LEFT_ENCODER_DISTANCE_PER_PULSE);
        leftEncoder.startLiveWindowMode();

        rightEncoder = new Encoder(Constants.Drive.RIGHT_ENCODER_A, Constants.Drive.RIGHT_ENCODER_B, false, Encoder.EncodingType.k4X);
        rightEncoder.setDistancePerPulse(Constants.Drive.RIGHT_ENCODER_DISTANCE_PER_PULSE);
        rightEncoder.startLiveWindowMode();

        turnReceiver = new PIDOutputReceiver();
        turnController = new PIDController(kPTurn, kITurn, kDTurn, kFTurn, ahrs, turnReceiver);
        turnController.setInputRange(-180, 180);
        turnController.setOutputRange(-Constants.Drive.TURN_MAX_POWER, Constants.Drive.TURN_MAX_POWER);
        turnController.setAbsoluteTolerance(Constants.Drive.TURN_TOLERANCE_DEGREES);
        turnController.setContinuous(true);
        turnController.startLiveWindowMode();

        leftReceiver = new PIDOutputReceiver();
        leftController = new PIDController(kPForward, kIForward, kDForward, kFForward, leftEncoder, leftReceiver);
        leftController.setOutputRange(-Constants.Drive.FORWARD_MAX_POWER, Constants.Drive.FORWARD_MAX_POWER);
        leftController.setPercentTolerance(Constants.Drive.FORWARD_TOLERANCE_METERS);
        leftController.setContinuous(true);
        leftController.startLiveWindowMode();

        rightReceiver = new PIDOutputReceiver();
        rightController = new PIDController(kPForward, kIForward, kDForward, kFForward, rightEncoder, rightReceiver);
        rightController.setOutputRange(-Constants.Drive.FORWARD_MAX_POWER, Constants.Drive.FORWARD_MAX_POWER)
        rightController.setPercentTolerance(Constants.Drive.FORWARD_TOLERANCE_METERS);
        rightController.setContinuous(true);
        rightController.startLiveWindowMode();

        LiveWindow.addActuator("Drive", "turnController", turnController);
        LiveWindow.addActuator("Drive", "leftController", leftController);
        LiveWindow.addActuator("Drive", "rightController", rightController);
        LiveWindow.addSensor("Drive", "Gyro", ahrs);
        LiveWindow.addSensor("Drive", "leftEncoder", leftEncoder);
        LiveWindow.addSensor("Drive", "rightEncoder", rightEncoder);
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
        SmartDashboard.putNumber("leftEncoder", leftEncoder.getDistance());
        SmartDashboard.putNumber("rightEncoder", rightEncoder.getDistance());
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
        turnController.disable();


        setLeftRightPower(signal.getLeftMotor(), signal.getRightMotor());
    }

    public synchronized void setAutonomousDriving() {
        currentState = DriveControlState.AUTONOMOUS_DRIVING;
    }

    public void setForwardSetpoint(double distance) {
        System.out.println("Set setpoint to " + distance);
        setAutonomousDriving();
        rightController.enable();
        leftController.enable();
        rightController.setSetpoint(distance);
        leftController.setSetpoint(distance);
    }

    public boolean goForward() {
        System.out.println("Moving rate of right: " + rightReceiver.getOutput() + " Distance: " + ahrs.getDisplacementX() + " Time: " + Timer.getFPGATimestamp());
        System.out.println("Moving rate of left: " + rightReceiver.getOutput() + " Distance: " + ahrs.getDisplacementX()+ " Time: " + Timer.getFPGATimestamp());
        rightController.updateTable();
        leftController.updateTable();
        if(rightReceiver.getOutput() == 0 || leftReceiver.getOutput() == 0){
            return true;
        }
        setLeftRightPower(leftReceiver.getOutput(), rightReceiver.getOutput());
        return false;
    }

    public void finishForward() {
        rightController.disable();
        leftController.disable();
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public synchronized void setAutonomousTurning() {
        currentState = DriveControlState.AUTONOMOUS_TURNING;
    }

    public void setAngleSetpoint(double angle) {
        System.out.println("Set setpoint to " + angle);
        setAutonomousTurning();
        turnController.enable();
        turnController.setSetpoint(angle);
    }

    public boolean turnAngle() {
        System.out.println("Turn rate: " + turnReceiver.getOutput() + " Angle: " + ahrs.getAngle() + " Time: " + Timer.getFPGATimestamp());
        turnController.updateTable();
        if (turnReceiver.getOutput() == 0) {
            return true;
        }
        setLeftRightPower(turnReceiver.getOutput(), -turnReceiver.getOutput());
        return false;
    }

    public void finishTurn() {
        turnController.disable();
        setOpenLoop(DriveSignal.NEUTRAL);
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
