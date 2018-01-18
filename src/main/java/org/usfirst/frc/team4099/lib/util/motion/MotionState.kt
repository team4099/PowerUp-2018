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
    fun vel2(): Double {
        return vel_ * vel_
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
            return t_
        }

        if (epsilonEquals(acc_,0.0, kEpsilon)) {
            val d_pos: Double = pos - pos_
            if(!epsilonEquals(vel_,0.0,kEpsilon) && Math.signum(d_pos) == Math.signum(vel_)) {
                return d_pos/vel_ + t_
            }
            return Double.NaN
        }

        val disc: Double = vel_*vel_ - 2.0*acc_*(pos_-pos)
        if (disc<0) {
            return Double.NaN
        }

        val sqrt_disc: Double = Math.sqrt(disc)
        val max_dt: Double = (-vel_+sqrt_disc)/acc_
        val min_dt: Double = (-vel_-sqrt_disc)/acc_
        if(min_dt>=0.0 && (max_dt < 0.0 || min_dt < max_dt)) {
            return t_ + min_dt
        }
        if(max_dt >=0) {
            return t_+max_dt
        }
        return Double.NaN
    }

    override fun toString(): String {
        return "(t="+t_+", pos="+pos_+", vel="+vel_+", acc="+acc_+")"
    }

    override fun equals(other: Any?): Boolean {
        return (other is MotionState) && equals(other as MotionState, kEpsilon)
    }

    fun equals(other: MotionState, epsilon: Double): Boolean {
        return coincident(other, epsilon) && epsilonEquals(acc_, other.acc_, epsilon)
    }

    fun coincident(other: MotionState): Boolean {
        return coincident(other, kEpsilon)
    }

    fun coincident(other: MotionState, epsilon: Double): Boolean {
        return epsilonEquals(t_, other.t_, epsilon) && epsilonEquals(pos_, other.pos_, epsilon) && epsilonEquals(vel_, other.vel_, epsilon)
    }

    fun flipped(): MotionState {
        return MotionState(t_, -pos_, -vel_, -acc_)
    }

    companion object {
        val kInvalidState: MotionState = MotionState(Double.NaN,Double.NaN,Double.NaN,Double.NaN)
    }
}