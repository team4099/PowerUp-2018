package org.usfirst.frc.team4099.robot.loops;

import org.usfirst.frc.team4099.robot.subsystems.Climber;
import org.usfirst.frc.team4099.robot.subsystems.Intake;

/** Manages the shutting off of components and subsystems when at risk of brownout.
 *  It does this through a multitude of steps:
 *  1. Constantly monitor the Battery voltage
 *  2.
 *  3.
 */
public class BrownoutDefender implements Loop {

    private Climber mClimber = Climber.getInstance();
    private Intake mIntake = Intake.getInstance();

    private static BrownoutDefender sInstance = new BrownoutDefender();

    public static BrownoutDefender getInstance() {
        return sInstance;
    }

    private BrownoutDefender() {}

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {
        if (mClimber.getClimberState().equals(Climber.ClimberState.CLIMBING)) {
            if (mIntake.getCompressor().getClosedLoopControl()) {
                mIntake.stopCompressor();
            }
        } else {
            if (!mIntake.getCompressor().getClosedLoopControl()) {
                mIntake.startCompressor();
            }
        }
    }

    @Override
    public void onStop() {

    }
}
