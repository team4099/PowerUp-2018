package org.usfirst.frc.team4099.auto.modes;

// go to the baseline

import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.ForwardAction;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;

/**
 * Created by plato2000 on 2/13/17.
 */
public class BaselineMode extends AutoModeBase {
    private final double initialForwardDistance;
    private double turningAngle;
    private boolean turnAround;

    public BaselineMode(AutonomousInitParameters initParameters, boolean turnAround) {
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getInitalForwardSeconds();
        this.turningAngle = initParameters.getTurnAngle().getDegrees();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ForwardAction(initialForwardDistance));
//        runAction(new TurnAction(Rotation2D.fromDegrees(60)));
    }


}
