package org.usfirst.frc.team4099.auto.actions;

import org.usfirst.frc.team4099.robot.subsystems.Drive;

/**
 * Created by plato2000 on 2/16/17.
 */
public class ForwardAction implements Action {
    private Drive mDrive;
    private double metersForward;
    private boolean isDone;

    public ForwardAction(double metersForward) {
        this.mDrive = Drive.getInstance();
        this.metersForward = metersForward;
        this.isDone = false;
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void update() {
        isDone = mDrive.goForward();
    }

    @Override
    public void done() {
        mDrive.finishForward();
    }

    @Override
    public void start() {
        mDrive.setForwardSetpoint(metersForward);
    }
}
