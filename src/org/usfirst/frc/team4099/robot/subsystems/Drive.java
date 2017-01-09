package org.usfirst.frc.team4099.robot.subsystems;

public class Drive implements Subsystem {

    private static Drive sInstance = new Drive();

    public Drive() {

    }

    public static Drive getInstance() {
        return sInstance;
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }
}
