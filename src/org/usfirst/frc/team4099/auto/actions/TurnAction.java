package org.usfirst.frc.team4099.auto.actions;

import org.usfirst.frc.team4099.lib.util.Rotation2D;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.subsystems.Drive;

/**
 * Created by plato2000 on 2/14/17.
 */
public class TurnAction implements Action {

    private Drive mDrive;
    private boolean isDone;
    private double finishAngle;

    public TurnAction(Rotation2D degreesToTurn) {
        this(degreesToTurn, false);
    }

    public TurnAction(Rotation2D degreesToTurn, boolean absolutePosition) {
        this.mDrive = Drive.getInstance();
        this.isDone = false;
        if(!absolutePosition) {
            finishAngle = degreesToTurn.getDegrees();
        } else {
            finishAngle = (180 - mDrive.getAHRS().getYaw() + degreesToTurn.getDegrees()) % 360 - 180;
        }
    }

    @Override
    public boolean isFinished() {
        return isDone || Math.abs(mDrive.getAHRS().getYaw() - finishAngle) < Constants.Drive.TURN_TOLERANCE_DEGREES;
    }

    @Override
    public void update() {
        isDone = mDrive.turnAngle();
        System.out.println("Still turning "  + isDone);
    }

    @Override
    public void done() {
        mDrive.finishTurn();
    }

    @Override
    public void start() {
        mDrive.setAngleSetpoint(finishAngle);
    }
}
