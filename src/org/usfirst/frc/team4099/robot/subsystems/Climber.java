package org.usfirst.frc.team4099.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils;
import org.usfirst.frc.team4099.lib.util.CrashTracker;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.loops.Loop;

public class Climber implements Subsystem {

    private static Climber sClimber = new Climber();
    private Talon climberTalon;
    private double climberPower;
    private ClimberState climberState = ClimberState.NOT_CLIMBING;

    public enum ClimberState {
        CLIMBING, NOT_CLIMBING;
    }

    private Climber() {
        climberTalon = new Talon(Constants.Climber.CLIMBER_TALON_ID);
    }

    public static Climber getInstance() {
        return sClimber;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("climberPower", climberPower);
    }

    @Override
    public synchronized void stop() {
        setClimberPower(0);
    }

    public ClimberState getClimberState() {
        return climberState;
    }

    @Override
    public void zeroSensors() {}

    public synchronized void setClimberPower(double climberPower) {
        this.climberPower = Math.abs(climberPower);
        climberTalon.set(climberPower);
    }

    public Loop getLoop() {
        return mLoop;
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            setClimberPower(0);
        }

        @Override
        public void onLoop() {
            synchronized (Climber.this) {
                switch (climberState) {
                    case CLIMBING:
                        setClimberPower(0.5);
                        break;
                    case NOT_CLIMBING:
                        setClimberPower(0);
                        break;
                    default:
                        CrashTracker.logMarker("REACHED ILLEGAL STATE IN CLIMBER");
                        setClimberPower(0);
                        climberState = ClimberState.NOT_CLIMBING;
                        break;
                }
            }
        }

        @Override
        public void onStop() {
            stop();
        }
    };
}
