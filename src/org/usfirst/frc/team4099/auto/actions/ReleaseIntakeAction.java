package org.usfirst.frc.team4099.auto.actions;

import org.usfirst.frc.team4099.robot.subsystems.Intake;

/**
 * Created by plato2000 on 2/14/17.
 */
public class ReleaseIntakeAction implements Action {
    private Intake mIntake;
    private boolean isDone;

    public ReleaseIntakeAction() {
        mIntake = Intake.getInstance();
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
//        mIntake.
    }
}
