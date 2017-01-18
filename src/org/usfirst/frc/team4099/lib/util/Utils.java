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

    public static boolean around(double value, double around, double tolerance) {
        return diff(value, around) <= tolerance;
    }

    public static boolean sameSign(double new_, double old_) {
        return (new_ >= 0 && old_ >= 0) || (new_ <= 0 && old_ <= 0);
    }

    public static int sign(double value) {
        return (value >= 0)? 1 : -1;
    }
}
