package org.usfirst.frc.team4099.auto.actions;

import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.robot.subsystems.Drive;

/**
 * Created by Oksana on 2/16/2017.
 */
public class MoveAction implements Action {
    private Drive mDrive;
    private double moveThisMuch;
    private boolean isDone;

    public MoveAction(double howMuchToMove){
        this.mDrive = Drive.getInstance();
        this.moveThisMuch = howMuchToMove;
        this.isDone = false;
    }
    public boolean isFinished(){ return isDone;}

    public void update(){  isDone = mDrive.goForward(); }

    public void done(){ mDrive.finishForward();}

    public void start(){ mDrive.setForwardSetpoint(moveThisMuch); }
}
