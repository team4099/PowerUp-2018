package org.usfirst.frc.team4099.lib.joystick;

import org.usfirst.frc.team4099.robot.loops.VoltageEstimator;

public class JoystickUtils {

    private JoystickUtils() {}

    /**
     * Transforms the joystick input from linear to cubic.
     * @param signal The raw input from the joystick
     * @param deadbandWidth The maximum output of the joystick that is still considered 0.
     * @return The transformed, potentially smoother control curve.
     */
    public static double deadband(double signal, double deadbandWidth) {
        // TODO: Implement MAX_OUTPUT limiter based on currentVoltage

        double currentVoltage = VoltageEstimator.getInstance().getAverageVoltage();
        double MAX_OUTPUT = 0.85;
        double alpha = 0.1;
        double beta = MAX_OUTPUT - alpha;

        signal = alpha * signal + beta * Math.pow(signal, 3);
        int sign = (signal > 0) ? 1 : -1;
        signal = Math.abs(signal);

        if (signal < deadbandWidth)
            return 0;

        return sign * (signal - deadbandWidth) / (1.0 - deadbandWidth);
    }

    public static double deadbandNoShape(double signal, double deadbandWidth) {
        double MAX_OUTPUT = 0.85;

        int sign = (signal > 0) ? 1 : -1;
        signal = Math.abs(signal);

        if (signal < deadbandWidth)
            return 0;

        return MAX_OUTPUT * sign * (signal - deadbandWidth) / (1.0 - deadbandWidth);
    }
}
