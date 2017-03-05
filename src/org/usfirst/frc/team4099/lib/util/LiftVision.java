package org.usfirst.frc.team4099.lib.util;

/**
 * Created by plato2000 on 3/4/17.
 */
public class LiftVision {

    private Rotation2D offsetAngle;
    private Rotation2D turnAngle;
    private double distanceToLift;

    public LiftVision(double offsetAngle, double turnAngle, double distanceToLift) {
        this.offsetAngle = Rotation2D.fromDegrees(offsetAngle);
        this.turnAngle = Rotation2D.fromDegrees(turnAngle);
        this.distanceToLift = distanceToLift;
    }

    public Rotation2D getOffsetAngle() {
        return new Rotation2D(offsetAngle);
    }

    public Rotation2D getTurnAngle() {
        return new Rotation2D(turnAngle);
    }

    public double getDistance() {
        return distanceToLift;
    }

}
