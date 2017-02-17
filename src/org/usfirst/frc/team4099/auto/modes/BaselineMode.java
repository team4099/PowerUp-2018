package org.usfirst.frc.team4099.auto.modes;

import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.TurnAction;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2D;

/**
 * Created by plato2000 on 2/13/17.
 */
public class BaselineMode extends AutoModeBase {
    private final double initialForwardDistance;
    private final boolean turnAround;

    public BaselineMode(AutonomousInitParameters initParameters, boolean turnAround) {
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new TurnAction(Rotation2D.fromDegrees(90)));
    }
}
