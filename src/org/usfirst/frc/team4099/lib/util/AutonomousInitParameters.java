package org.usfirst.frc.team4099.lib.util;

/**
 * Created by plato2000 on 2/13/17.
 */
public class AutonomousInitParameters {
    private double distanceInMeters;
    private Rotation2D turnAngle;
    private int trackID;

    public AutonomousInitParameters(double forwardDistance, Rotation2D turnAngle, int trackID) {
        this.distanceInMeters = forwardDistance;
        this.turnAngle = turnAngle;
        this.trackID = trackID;
    }

    public double getDistanceInMeters() {
        return distanceInMeters;
    }

    public Rotation2D getTurnAngle() {
        return turnAngle;
    }

    public int getTrackID() {
        return trackID;
    }

}
