package org.usfirst.frc.team4099.lib.joystick;

public class JoystickUtils {

    public static double deadband(double signal) {
        double DB_POWER = 2.15;
        double DB_LIMIT = 0.1;
        double _limit = Math.pow(DB_LIMIT, DB_POWER);  // deadband limit; internal to function

        int sign = (signal > 0) ? 1 : -1;
        signal = Math.abs(signal);
        signal = Math.pow(signal, 2.15);

        if (signal < _limit)
            return 0;

        double r_signal = sign * (signal - _limit) / (1.0 - _limit);

        return r_signal;
    }

}
