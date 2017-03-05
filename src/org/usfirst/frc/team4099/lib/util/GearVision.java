package org.usfirst.frc.team4099.lib.util;

/**
 * Created by plato2000 on 3/4/17.
 */
public class GearVision {
    private Rotation2D turnToGear;
    private double distanceToGear;

    public GearVision(double angleToGear, double distanceToGear) {
        turnToGear = Rotation2D.fromDegrees(angleToGear);
        distanceToGear = distanceToGear;
    }

    public Rotation2D getTurnAngle() {
        return new Rotation2D(turnToGear);
    }

    public double getDistance() {
        return distanceToGear;
    }

}
