package org.usfirst.frc.team4099.lib.drive;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Created by plato2000 on 2/14/17.
 */
public class PIDOutputReceiver implements PIDOutput {
    private double output = 0;

    @Override
    public void pidWrite(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }
}
