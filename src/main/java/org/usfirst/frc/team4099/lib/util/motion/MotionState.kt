package org.usfirst.frc.team4099.lib.util.motion

import org.usfirst.frc.team4099.lib.util.Utils.epsilonEquals
import org.usfirst.frc.team4099.lib.util.motion.motionUtil.kEpsilon

class MotionState {
    protected var t_: Double = 0.0
    protected var pos_: Double = 0.0
    protected var vel_: Double = 0.0
    protected var acc_: Double = 0.0

    constructor(t: Double, pos: Double, vel: Double, acc: Double) {
        t_ = t
        pos_ = pos
        vel_ = vel
        acc_ = acc
    }

    constructor(other: MotionState) {
        MotionState(other.t_, other.pos_, other.vel_, other.acc_)
    }

    fun t(): Double {
        return t_
    }

    fun pos(): Double {
        return pos_
    }

    fun vel(): Double {
        return vel_
    }

    fun acc(): Double {
        return acc_
    }

    fun extrapolate(t: Double): MotionState {
        return extrapolate(t, acc_)
    }

    fun extrapolate(t: Double, acc: Double): MotionState {
        val dt: Double = t - t_
        return MotionState(t_, pos_+vel_*dt+0.5*acc_*dt*dt, vel_+acc_*dt, acc_)
    }

    fun nextTimeAtPos(pos: Double): Double {
        if (epsilonEquals(pos, this.pos, kEpsilon)) {

        }
    }

    companion object {
        val kInvalidState: MotionState = MotionState(Double.NaN,Double.NaN,Double.NaN,Double.NaN)
    }
}