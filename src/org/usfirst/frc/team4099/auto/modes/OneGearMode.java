package org.usfirst.frc.team4099.auto.modes;

import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2d;

/**
 * Created by plato2000 on 2/13/17.
 */
public class OneGearMode extends AutoModeBase {

    private final double initialForwardDistance;
    private final Rotation2d initialTurn;
    private final boolean goToBaseline;
    private final boolean turnAround;

    public OneGearMode(AutonomousInitParameters initParameters, boolean goToBaseline, boolean turnAround) {
        this.goToBaseline = goToBaseline;
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
        this.initialTurn = initParameters.getTurnAngle();
    }

    @Override
    protected void routine() throws AutoModeEndedException {

    }
}
