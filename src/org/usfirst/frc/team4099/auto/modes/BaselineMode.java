package org.usfirst.frc.team4099.auto.modes;

// go to the baseline

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.*;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2D;

/**
 * Created by plato2000 on 2/13/17.
 */
public class BaselineMode extends AutoModeBase {
    private final double initialForwardDistance;
    private double turningAngle;
    private boolean turnAround;

    public BaselineMode(AutonomousInitParameters initParameters, boolean turnAround) {
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
        this.turningAngle = initParameters.getTurnAngle().getDegrees();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        if (turningAngle > 0 || turningAngle < 0){
            runAction(new ForwardAction(initialForwardDistance + 1));
        }else{
            Rotation2D turn = Rotation2D.fromDegrees(90);
            if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue){
                turn = turn.inverse();
            }
            runAction(new TurnAction(turn));
            runAction(new ForwardAction(1.37)); // distance in meters
            runAction(new TurnAction(turn.inverse()));
            runAction(new ForwardAction(initialForwardDistance + 1));
        }

    }


}
