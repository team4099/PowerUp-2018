package org.usfirst.frc.team4099.robot.subsystems;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.lib.drive.PIDOutputReceiver;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.loops.Loop;

public class Drive implements Subsystem {

    private static Drive sInstance = new Drive();
    private CANSpeedController leftFrontSRX, leftBackSRX, rightFrontSRX, rightBackSRX;
    private AHRS ahrs;
    private DriveControlState currentState = DriveControlState.OPEN_LOOP;

    public enum DriveControlState {
        OPEN_LOOP,
        AUTONOMOUS_TURNING,
        AUTONOMOUS_DRIVING
    }

    private PIDController turnController;
    private PIDController leftController;
    private PIDController rightController;

    private PIDOutputReceiver turnReceiver;
    private PIDOutputReceiver leftReceiver;
    private PIDOutputReceiver rightReceiver;

    public Encoder leftEncoder;
    public Encoder rightEncoder;

//    private LimitedQueue<Double> lastTurnErrors = new LimitedQueue<>(10);
//    private LimitedQueue<Double> lastForwardErrors = new LimitedQueue<>(10);

    private Drive() {
        leftFrontSRX = new CANTalon(Constants.Drive.LEFT_FRONT_ID);
        leftBackSRX = new CANTalon(Constants.Drive.LEFT_BACK_ID);

        rightFrontSRX = new CANTalon(Constants.Drive.RIGHT_FRONT_ID);
        rightBackSRX = new CANTalon(Constants.Drive.RIGHT_BACK_ID);

        ahrs = new AHRS(SPI.Port.kMXP);

        leftEncoder = new Encoder(Constants.Drive.LEFT_ENCODER_A, Constants.Drive.LEFT_ENCODER_B, true, Encoder.EncodingType.k4X);
        leftEncoder.setSamplesToAverage(Constants.Drive.ENCODER_SAMPLES_TO_AVERAGE);
        leftEncoder.setDistancePerPulse(Constants.Drive.LEFT_ENCODER_INCHES_PER_PULSE);

        rightEncoder = new Encoder(Constants.Drive.RIGHT_ENCODER_A, Constants.Drive.RIGHT_ENCODER_B, false, Encoder.EncodingType.k4X);
        rightEncoder.setSamplesToAverage(Constants.Drive.ENCODER_SAMPLES_TO_AVERAGE);
        rightEncoder.setDistancePerPulse(Constants.Drive.RIGHT_ENCODER_INCHES_PER_PULSE);

        turnReceiver = new PIDOutputReceiver();
        turnController = new PIDController(Constants.Gains.TURN_P, Constants.Gains.TURN_I, Constants.Gains.TURN_D, Constants.Gains.TURN_F, ahrs, turnReceiver);
        turnController.setInputRange(-180, 180);
        turnController.setOutputRange(-Constants.Drive.AUTO_TURN_MAX_POWER, Constants.Drive.AUTO_TURN_MAX_POWER);
        turnController.setAbsoluteTolerance(Constants.Drive.TURN_TOLERANCE_DEGREES);
        turnController.setContinuous(true);

        leftReceiver = new PIDOutputReceiver();
        leftController = new PIDController(Constants.Gains.FORWARD_P, Constants.Gains.FORWARD_I, Constants.Gains.FORWARD_D, Constants.Gains.FORWARD_F, leftEncoder, leftReceiver);
        leftController.setInputRange(-200, 200);
        leftController.setOutputRange(-Constants.Drive.AUTO_FORWARD_MAX_POWER, Constants.Drive.AUTO_FORWARD_MAX_POWER);
        leftController.setAbsoluteTolerance(Constants.Drive.FORWARD_TOLERANCE_INCHES);
        leftController.setContinuous(false);

        rightReceiver = new PIDOutputReceiver();
        rightController = new PIDController(Constants.Gains.FORWARD_P, Constants.Gains.FORWARD_I, Constants.Gains.FORWARD_D, Constants.Gains.FORWARD_F, rightEncoder, rightReceiver);
        rightController.setInputRange(-200, 200);
        rightController.setOutputRange(-Constants.Drive.AUTO_FORWARD_MAX_POWER, Constants.Drive.AUTO_FORWARD_MAX_POWER);
        rightController.setAbsoluteTolerance(Constants.Drive.FORWARD_TOLERANCE_INCHES);
        rightController.setContinuous(false);

        this.zeroSensors();
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
            SmartDashboard.putNumber("gyro", this.getAHRS().getYaw());
        }
        else {
            SmartDashboard.putNumber("gyro", -31337);
        }
        SmartDashboard.putNumber("leftTalon", leftFrontSRX.get());
        SmartDashboard.putNumber("rightTalon", rightFrontSRX.get());
        SmartDashboard.putNumber("leftEncoder", leftEncoder.getDistance());
        SmartDashboard.putNumber("rightEncoder", rightEncoder.getDistance());
        SmartDashboard.putNumber("leftEncoderRaw", leftEncoder.get());
        SmartDashboard.putNumber("rightEncoderRaw", rightEncoder.get());
    }

    public void startLiveWindowMode() {
//        System.out.println("do a potato");
//        LiveWindow.addActuator("Drive", "turnController", turnController);
//        LiveWindow.addActuator("Drive", "leftController", leftController);
//        LiveWindow.addActuator("Drive", "rightController", rightController);
//        LiveWindow.addSensor("Drive", "Gyro", ahrs);
//        LiveWindow.addSensor("Drive", "leftEncoder", leftEncoder);
//        LiveWindow.addSensor("Drive", "rightEncoder", rightEncoder);
//        leftEncoder.startLiveWindowMode();
//        rightEncoder.startLiveWindowMode();
//        leftController.startLiveWindowMode();
//        rightController.startLiveWindowMode();

    }

    public void stopLiveWindowMode() {
        leftEncoder.stopLiveWindowMode();
        rightEncoder.stopLiveWindowMode();
        leftController.stopLiveWindowMode();
        rightController.stopLiveWindowMode();
    }

    public void updateLiveWindowTables() {
//        leftEncoder.updateTable();
//        rightEncoder.updateTable();
//        leftController.updateTable();
//        rightController.updateTable();
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
//        System.out.println("Left: " + left + "Right: " + right);
        leftFrontSRX.set(-left);
        leftBackSRX.set(-left);
        rightFrontSRX.set(right);
        rightBackSRX.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (currentState != DriveControlState.OPEN_LOOP) {
            currentState = DriveControlState.OPEN_LOOP;
        }
        turnController.disable();
        leftController.disable();
        rightController.disable();

        setLeftRightPower(signal.getLeftMotor(), signal.getRightMotor());
    }

    public synchronized void setAutonomousDriving() {
        currentState = DriveControlState.AUTONOMOUS_DRIVING;
    }

    public void setForwardSetpoint(double distance) {
        System.out.println("Set setpoint to " + distance);
        setAutonomousDriving();
        leftController.enable();
        rightController.enable();
        leftController.setSetpoint(distance);
        rightController.setSetpoint(distance);
    }

    public boolean goForward() {
//        System.out.println("Moving rate of right: " + rightReceiver.getOutput() + " Distance: " + rightEncoder.pidGet() + " Time: " + Timer.getFPGATimestamp());
//        System.out.println("Moving rate of left: " + leftReceiver.getOutput() + " Distance: " + leftEncoder.pidGet() + " Time: " + Timer.getFPGATimestamp());
//        System.out.println("Left Setpoint: " + leftController.getSetpoint());
//        System.out.println("Right Setpoint: " + rightController.getSetpoint());

//        double leftError = Math.abs(leftController.getSetpoint())
//        lastForwardErrors.add(Math.abs(left))
        if(leftController.onTarget() || currentState != DriveControlState.AUTONOMOUS_DRIVING){
            return true;
        }
        setLeftRightPower(-leftReceiver.getOutput(), -leftReceiver.getOutput() - rightReceiver.getOutput() * .15);
        return false;
    }

    public void finishForward() {
        leftController.disable();
        rightController.disable();
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
        System.out.println("Turn setpoint: " + turnController.getSetpoint() + " Angle: " + ahrs.getYaw() + " Time: " + Timer.getFPGATimestamp());
//        lastTurnErrors.add(Math.abs(turnController.getSetpoint() - ahrs.getYaw()));
        if (turnController.onTarget() || currentState != DriveControlState.AUTONOMOUS_TURNING) {
            return true;
        }
        setLeftRightPower(-turnReceiver.getOutput(), turnReceiver.getOutput());
        return false;
    }

    public void finishTurn() {
        System.out.println("Finished turn");
        turnController.disable();
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public void arcadeDrive(double outputMagnitude, double curve) {
        final double leftOutput;
        final double rightOutput;

        if (curve < 0) {
            double value = Math.log(-curve);
            double ratio = (value - .5) / (value + .5);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude / ratio;
            rightOutput = outputMagnitude;
        } else if (curve > 0) {
            double value = Math.log(curve);
            double ratio = (value - .5) / (value + .5);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude / ratio;
        } else {
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude;
        }
        setLeftRightPower(leftOutput, rightOutput);
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
