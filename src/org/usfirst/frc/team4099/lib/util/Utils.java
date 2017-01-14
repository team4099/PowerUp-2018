package org.usfirst.frc.team4099.lib.util;

public class Utils {
    private Utils() {}

    /**
     * Limits the given input to the given magnitude.
     * @param v         value to limit
     * @param limit     limited magnitude
     * @return          the limited value
     */
    public static double limit(double v, double limit) {
        if (Math.abs(v) < limit)
            return v;
        if (v < 0)
            return -limit;
        else
            return limit;
    }


    public static double diff(double current, double prev) {
        return Math.abs(current - prev);
    }

}
