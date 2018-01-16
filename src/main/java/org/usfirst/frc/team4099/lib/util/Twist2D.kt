package org.usfirst.frc.team4099.lib.util

class Twist2D {

    protected final val kIdentity : Twist2D = Twist2D(0.0,0.0,0.0)

    public final fun identity() : Twist2D {
        return kIdentity
    }

    private var dx : Double = 0.0
    private var dy : Double = 0.0
    private var dtheta : Double = 0.0//in radians

    public constructor(dx : Double, dy : Double, dtheta : Double){
        this.dx = dx
        this.dy = dy
        this.dtheta = dtheta

    }

    public fun scaled(scale : Double) : Twist2D{
        return Twist2D(dx * scale, dy * scale, dtheta * scale)
    }
}