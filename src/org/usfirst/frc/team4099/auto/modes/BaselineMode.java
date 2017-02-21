package org.usfirst.frc.team4099.auto.modes;

// go to the baseline

import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.*;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2D;

/**
 * Created by plato2000 on 2/13/17.
 */
public class BaselineMode extends AutoModeBase {
    private final double initialForwardDistance;
    private final boolean turnAround;
    private double turningAngle;

    public BaselineMode(AutonomousInitParameters initParameters, boolean turnAround) {
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
        this.turningAngle = initParameters.getTurnAngle().getDegrees();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        if (turningAngle > 0 || turningAngle < 0){
            runAction(new ForwardAction(initialForwardDistance + 2));
        }else{
            runAction(new ForwardAction(1));
            runAction(new TurnAction(Rotation2D.fromDegrees(90)));
            runAction(new ForwardAction(1.37)); // distance in meters
            runAction(new TurnAction(Rotation2D.fromDegrees(-90)));
            runAction(new ForwardAction(initialForwardDistance + 1));
        }

    }


}
