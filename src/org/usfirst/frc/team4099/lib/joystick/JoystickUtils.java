package org.usfirst.frc.team4099.lib.joystick;

public class JoystickUtils {

    private JoystickUtils() {}

    public static double deadband(double signal, double limit) {
        double DB_POWER = 2.15;
        double DB_LIMIT = limit;
        double _limit = Math.pow(DB_LIMIT, DB_POWER);  // deadband limit; internal to function
        double DB_DAMPEN = 1.0;

        int sign = (signal > 0) ? 1 : -1;
        signal = Math.abs(signal);
        signal = Math.pow(signal, 2.15);

        if (signal < _limit)
            return 0;

        return sign * DB_DAMPEN * (signal - _limit) / (1.0 - _limit);
    }

}
