package org.usfirst.frc.team4099.auto.actions;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4099.lib.util.Utils;
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
    private double startAngle;
    private boolean resetGyro;
    private boolean done;

    public ForwardAction(double secondsToMove, boolean slowMode, boolean resetGyro) {
        this(secondsToMove);
        if(slowMode) {
            this.power = Constants.Drive.AUTO_FORWARD_SLOW_POWER;
        }
        this.resetGyro = resetGyro;
    }

    public ForwardAction(double secondsToMove){
        this.mDrive = Drive.getInstance();
        this.secondsToMove = Math.abs(secondsToMove);
        direction = (int) secondsToMove / (int) this.secondsToMove;
        this.power = Constants.Drive.AUTO_FORWARD_MAX_POWER;
        done = false;
        this.resetGyro = false;
    }
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= secondsToMove || done;
    }

    public void update() {
        AHRS ahrs = mDrive.getAHRS();
        double yaw = ahrs.getYaw();
//        double correctionAngle = Math.IEEEremainder(yaw - startAngle, 360);
        double correctionAngle = startAngle - yaw;
        if(Math.abs(correctionAngle) > 30) {
            done = true;
            return;
        }
        mDrive.arcadeDrive(-power * direction, correctionAngle * Constants.Drive.FORWARD_KP * direction);
//        System.out.println("yaw: " + yaw);
        System.out.println("correctionAngle: " + correctionAngle);
    }

    public void done() {
        mDrive.finishForward();
        System.out.println("------- END FORWARD -------");
    }

    @Override
    public void start(){
        if(resetGyro) {
            while(!Utils.around(mDrive.getAHRS().getYaw(), 0, 1)) {
                mDrive.getAHRS().zeroYaw();
            }
            Timer.delay(1);
        }
        startAngle = mDrive.getAHRS().getYaw();
        System.out.println("------- NEW START AUTONOMOUS RUN -------");
        System.out.println("Starting angle: " + startAngle);
        startTime = Timer.getFPGATimestamp();
    }
}
