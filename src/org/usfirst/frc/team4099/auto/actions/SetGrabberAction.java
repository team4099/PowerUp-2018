package org.usfirst.frc.team4099.auto.actions;

import org.usfirst.frc.team4099.robot.subsystems.Intake;

/**
 * Created by plato2000 on 2/14/17.
 */
public class SetGrabberAction implements Action {
    private Intake mIntake;
    private boolean isDone;
    private Intake.GrabberPosition positionToSet;

    public SetGrabberAction(Intake.GrabberPosition position) {
        mIntake = Intake.getInstance();
        positionToSet = position;
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mIntake.updateIntake(mIntake.getIntakePosition(), positionToSet);
        isDone = true;
    }
}
