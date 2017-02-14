package org.usfirst.frc.team4099.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils;
import org.usfirst.frc.team4099.robot.loops.Loop;

/**
 * Created by plato2000 on 2/13/17.
 */
public class Climber implements Subsystem {

    public static Climber sClimber = new Climber();
    private Talon climberTalon;
    private double currentPower;


    private Climber() {
        this.climberTalon = new Talon(5);
    }

    public static Climber getInstance() {
        return sClimber;
    }

    @Override
    public void outputToSmartDashboard() {
//        SmartDashboard.putNumber("climberPower", );

    }

    @Override
    public synchronized void stop() {
        setClimberPower(0);
        climberTalon.set(0);
    }

    @Override
    public void zeroSensors() {}

    public synchronized void setClimberPower(double climberPower) {
        this.currentPower = Math.abs(JoystickUtils.deadband(climberPower, .2));
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
                climberTalon.set(currentPower);
            }
        }

        @Override
        public void onStop() {
            stop();
        }
    };
}
