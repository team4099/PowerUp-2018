package org.usfirst.frc.team4099.auto.actions;

import org.usfirst.frc.team4099.lib.util.Rotation2D;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.subsystems.Drive;

/**
 * Created by plato2000 on 2/14/17.
 */
public class TurnAction implements Action {

    private Drive mDrive;
    private double degreesToTurn;
    private boolean isDone;
    private double finishAngle;

    public TurnAction(Rotation2D degreesToTurn) {
        this.mDrive = Drive.getInstance();
        this.degreesToTurn = degreesToTurn.getDegrees();
        this.isDone = false;
    }

    @Override
    public boolean isFinished() {
        return isDone || Math.abs(mDrive.getAHRS().getYaw() - finishAngle) < Constants.Drive.TURN_TOLERANCE_DEGREES;
    }

    @Override
    public void update() {
        isDone = mDrive.turnAngle();
    }

    @Override
    public void done() {
        mDrive.finishTurn();
    }

    @Override
    public void start() {
        finishAngle = Math.IEEEremainder(mDrive.getAHRS().getYaw() + degreesToTurn, 360);
        mDrive.setAngleSetpoint(finishAngle);
    }
}
