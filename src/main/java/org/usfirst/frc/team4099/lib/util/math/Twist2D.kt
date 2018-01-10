package org.usfirst.frc.team4099.lib.util.math

class Twist2D {

    protected var dx_: Double = 0.0;
    protected var dy_: Double = 0.0;
    protected var dtheta_: Double = 0.0; //Radians or WILL NOT WORK

    @JvmOverloads constructor(dx: Double = 0.0, dy: Double = 0.0, dtheta: Double=0.0) {
        dx_ = dx;
        dy_ = dy;
        dtheta_ = dtheta;
    }

    fun dx(): Double {
        return dx_;
    }

    fun dy(): Double {
        return dy_;
    }

    fun dtheta(): Double {
        return dtheta_;
    }

    fun scaled(scale: Double): Twist2D {
        return Twist2D(dx_ * scale, dy_ * scale, dtheta_ * scale);
    }
}