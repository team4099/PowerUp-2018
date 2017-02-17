package org.usfirst.frc.team4099.auto.modes;

import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2D;

/**
 * Created by plato2000 on 2/13/17.
 */
public class TwoGearMode extends AutoModeBase {
    private final double initialForwardDistance;
    private final Rotation2D initialTurn;
    private final boolean goToBaseline;
    private final boolean turnAround;

    public TwoGearMode(AutonomousInitParameters initParameters, boolean goToBaseline, boolean turnAround) {
        this.goToBaseline = goToBaseline;
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
        this.initialTurn = initParameters.getTurnAngle();
    }

    @Override
    protected void routine() throws AutoModeEndedException {

    }
}
