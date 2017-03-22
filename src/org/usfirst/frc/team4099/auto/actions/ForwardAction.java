package org.usfirst.frc.team4099.auto.actions;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.subsystems.Drive;

/**
 * Created by Oksana on 2/16/2017.
 */
public class ForwardAction implements Action {
    private Drive mDrive;
    private double secondsToMove;
    private double startTime;
    private int direction;
    private double power;

    public ForwardAction(double secondsToMove, boolean slowMode) {
        this(secondsToMove);
        if(slowMode) {
            this.power = Constants.Drive.AUTO_FORWARD_SLOW_POWER;
        }
    }

    public ForwardAction(double secondsToMove){
        this.mDrive = Drive.getInstance();
        this.secondsToMove = Math.abs(secondsToMove);
        startTime = Timer.getFPGATimestamp();
        direction = (int) secondsToMove / (int) this.secondsToMove;
        this.power = Constants.Drive.AUTO_FORWARD_MAX_POWER;
    }
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= secondsToMove;
    }

    public void update() {
        AHRS ahrs = mDrive.getAHRS();
        double yaw = ahrs.getYaw();
        mDrive.arcadeDrive(-power * direction, -yaw * Constants.Drive.FORWARD_KP * direction);
//        System.out.println("yaw: " + yaw);
    }

    public void done(){ mDrive.finishForward();}

    public void start(){
        mDrive.getAHRS().reset();
//        Timer.delay(.5);
    }
}
