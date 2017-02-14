package org.usfirst.frc.team4099.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.loops.Loop;

public class Intake implements Subsystem {
    private static Intake sIntake = new Intake();

    private DoubleSolenoid upAndDown;
    private DoubleSolenoid gearGrabber;

    private boolean lastToggleUp;
    private boolean lastToggleGrab;

    private Compressor compressor;

    public enum GrabberPosition {
        OPEN, CLOSED;
    }
    public enum IntakePosition {
        UP, DOWN;
    }

    private IntakePosition intakePosition = IntakePosition.UP;
    private GrabberPosition grabberPosition = GrabberPosition.CLOSED;

    private Intake() {
        compressor = new Compressor();
        this.upAndDown = new DoubleSolenoid(
                Constants.Intake.UP_DOWN_SOLENOID_FORWARD,
                Constants.Intake.UP_DOWN_SOLENOID_REVERSE);
        this.gearGrabber = new DoubleSolenoid(
                Constants.Intake.GRAB_SOLENOID_FORWARD,
                Constants.Intake.GRAB_SOLENOID_REVERSE);
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
        if(intakePosition != null)
            SmartDashboard.putBoolean("Intake.isUp", intakePosition.equals(IntakePosition.UP));
        if(grabberPosition != null)
        SmartDashboard.putBoolean("Intake.isClosed", grabberPosition.equals(GrabberPosition.CLOSED));
        SmartDashboard.putNumber("Compressor Current Draw", compressor.getCompressorCurrent());
        SmartDashboard.putBoolean("Pressure Switch Value", compressor.getPressureSwitchValue());
    }

    @Override
    public synchronized void stop() {
        intakePosition = IntakePosition.UP;
        grabberPosition = GrabberPosition.CLOSED;
        setIntakePositions();
    }

    @Override
    public void zeroSensors() {}

    public synchronized void updateIntakePositions(boolean toggleUp, boolean toggleGrab) {
        if(toggleUp && !lastToggleUp) {
            if(intakePosition.equals(IntakePosition.DOWN)) {
                intakePosition = IntakePosition.UP;
            } else {
                intakePosition = IntakePosition.DOWN;
            }
        }
        if(toggleGrab && !lastToggleGrab) {
            if(grabberPosition.equals(GrabberPosition.OPEN)) {
                grabberPosition = GrabberPosition.CLOSED;
            } else {
                grabberPosition = GrabberPosition.OPEN;
            }
        }

        lastToggleGrab = toggleGrab;
        lastToggleUp = toggleUp;
    }

    private synchronized void updateIntakePositions(IntakePosition intakePosition, GrabberPosition grabberPosition) {
        this.intakePosition = intakePosition;
        this.grabberPosition = grabberPosition;
        setIntakePositions();
    }

    private synchronized void setIntakePositions() {
        switch(intakePosition) {
            case UP:
                gearGrabber.set(DoubleSolenoid.Value.kReverse);
                break;
            case DOWN:
                gearGrabber.set(DoubleSolenoid.Value.kForward);
                break;
        }
        switch(grabberPosition) {
            case OPEN:
                upAndDown.set(DoubleSolenoid.Value.kReverse);
                break;
            case CLOSED:
                upAndDown.set(DoubleSolenoid.Value.kForward);
        }
    }

    public Loop getLoop() {
        return mLoop;
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            updateIntakePositions(IntakePosition.UP, GrabberPosition.CLOSED);
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