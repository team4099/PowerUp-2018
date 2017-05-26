package org.usfirst.frc.team4099.auto.modes;

import org.usfirst.frc.team4099.auto.AutoModeEndedException;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot
 * standstill
 */
public class StandStillMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting Stand Still Mode... Done!");
    }
}
