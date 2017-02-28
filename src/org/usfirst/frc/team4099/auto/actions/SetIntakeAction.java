package org.usfirst.frc.team4099.auto.actions;

import org.usfirst.frc.team4099.robot.subsystems.Intake;

/**
 * Created by plato2000 on 2/14/17.
 */
public class SetIntakeAction implements Action {
    private Intake mIntake;
    private boolean isDone;
    private Intake.IntakePosition positionToSet;

    public SetIntakeAction(Intake.IntakePosition position) {
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
        mIntake.updateIntake(positionToSet);
        isDone = true;
    }
}
