package org.usfirst.frc.team4099.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.robot.loops.Loop;

/**
 * Created by plato2000 on 2/7/17.
 */
public class Intake implements Subsystem {
    public static Intake sIntake = new Intake();

    private DoubleSolenoid upAndDown;
    private DoubleSolenoid gearGrabber;

    public enum GrabberPosition {
        OPEN, CLOSED;
    }
    public enum IntakePosition {
        UP, DOWN;
    }

    private IntakePosition intakePosition;
    private GrabberPosition grabberPosition;

    private boolean lastToggleUp;
    private boolean lastToggleGrab;

    private Intake() {
        this.upAndDown = new DoubleSolenoid(1, 0);
        this.gearGrabber = new DoubleSolenoid(2, 3);
        this.intakePosition = IntakePosition.UP;
        this.grabberPosition = GrabberPosition.CLOSED;
    }

    public static Intake getInstance() {
        return sIntake;
    }

    @Override
    public void outputToSmartDashboard() {
        if(intakePosition != null)
            SmartDashboard.putBoolean("Intake.isUp", intakePosition.equals(IntakePosition.UP));
        if(grabberPosition != null)
        SmartDashboard.putBoolean("Intake.isClosed", grabberPosition.equals(GrabberPosition.CLOSED));
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