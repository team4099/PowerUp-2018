package org.usfirst.frc.team4099.lib.joystick;

public class JoystickUtils {

    private JoystickUtils() {}

    public static double deadband(double signal, double limit) {
        double DB_POWER = 2.8;
        double DB_LIMIT = limit;
        double _limit = Math.pow(DB_LIMIT, DB_POWER);  // deadband limit; internal to function

        int sign = (signal > 0) ? 1 : -1;
        signal = Math.abs(signal);
        signal = Math.pow(signal, DB_POWER);

        if (signal < _limit)
            return 0;

        return sign * (signal - _limit) / (1.0 - _limit);
    }

}
