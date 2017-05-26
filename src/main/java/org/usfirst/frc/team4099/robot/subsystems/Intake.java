package org.usfirst.frc.team4099.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.loops.Loop;

public class Intake implements Subsystem {
    private static Intake sIntake = new Intake();

    private DoubleSolenoid upAndDown;
    private DoubleSolenoid gearGrabber;

    private boolean lastToggleIntake;

    private Compressor compressor;

    private double upTime;

    public enum IntakePosition {
        UP_AND_CLOSED, UP_AND_OPEN, DOWN_AND_OPEN, DOWN_AND_CLOSED;
    }

    private IntakePosition intakePosition;

    private Intake() {
        compressor = new Compressor();
        this.upAndDown = new DoubleSolenoid(
                Constants.Intake.UP_DOWN_SOLENOID_FORWARD,
                Constants.Intake.UP_DOWN_SOLENOID_REVERSE);
        this.gearGrabber = new DoubleSolenoid(
                Constants.Intake.GRAB_SOLENOID_FORWARD,
                Constants.Intake.GRAB_SOLENOID_REVERSE);

        intakePosition = IntakePosition.UP_AND_CLOSED;
        upTime = -100;
    }

    public static Intake getInstance() {
        return sIntake;
    }

    public void stopCompressor() {
        compressor.stop();
    }

    public void startCompressor() {
        compressor.start();
    }

    public Compressor getCompressor() {
        return compressor;
    }

    @Override
    public void outputToSmartDashboard() {
        if(intakePosition != null) {
            SmartDashboard.putBoolean("Intake.isUp", !intakePosition.equals(IntakePosition.DOWN_AND_OPEN));
            SmartDashboard.putBoolean("Intake.isClosed", intakePosition.equals(IntakePosition.UP_AND_CLOSED));
        }
        SmartDashboard.putNumber("Compressor Current Draw", compressor.getCompressorCurrent());
        SmartDashboard.putBoolean("Pressure Switch Value", compressor.getPressureSwitchValue());
    }

    @Override
    public synchronized void stop() {
        intakePosition = IntakePosition.UP_AND_CLOSED;
        setIntakePositions();
    }

    @Override
    public void zeroSensors() {}

    public synchronized void updateIntake(boolean toggleIntake) {
        if(toggleIntake && !lastToggleIntake) {
            if(intakePosition.equals(IntakePosition.DOWN_AND_OPEN)) {
                this.upTime = Timer.getFPGATimestamp();
                intakePosition = IntakePosition.UP_AND_CLOSED;
            } else if(intakePosition.equals(IntakePosition.UP_AND_CLOSED)){
                intakePosition = IntakePosition.UP_AND_OPEN;
            } else if(intakePosition.equals(IntakePosition.UP_AND_OPEN)){
                intakePosition = IntakePosition.DOWN_AND_OPEN;
            } else if(intakePosition.equals(IntakePosition.DOWN_AND_CLOSED)) {
                intakePosition = IntakePosition.UP_AND_CLOSED;
            }
        }

        lastToggleIntake = toggleIntake;
    }

    public synchronized void updateIntake(IntakePosition intakePosition) {
        this.intakePosition = intakePosition;
        setIntakePositions();
    }

    public synchronized IntakePosition getIntakePosition() {
        return this.intakePosition;
    }

    private synchronized void setIntakePositions() {
        switch(intakePosition) {
            case UP_AND_CLOSED:
                if(upTime == -1) {
                    upTime = Timer.getFPGATimestamp();
                }
                gearGrabber.set(DoubleSolenoid.Value.kReverse);
                if(Timer.getFPGATimestamp() - upTime > .4) {
                    upAndDown.set(DoubleSolenoid.Value.kReverse);
                }
                break;
            case UP_AND_OPEN:
                gearGrabber.set(DoubleSolenoid.Value.kForward);
                upAndDown.set(DoubleSolenoid.Value.kReverse);
                upTime = -1;
                break;
            case DOWN_AND_OPEN:
                upAndDown.set(DoubleSolenoid.Value.kForward);
                gearGrabber.set(DoubleSolenoid.Value.kForward);
                upTime = -1;
                break;
            case DOWN_AND_CLOSED:
                upAndDown.set(DoubleSolenoid.Value.kForward);
                gearGrabber.set(DoubleSolenoid.Value.kReverse);
                upTime = -1;
                break;
        }
    }

    public Loop getLoop() {
        return mLoop;
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            updateIntake(IntakePosition.UP_AND_CLOSED);
        }

        @Override
        public void onLoop() {
            synchronized (Intake.this) {
                setIntakePositions();
            }
        }

        @Override
        public void onStop() {
            stop();
        }
    };

}