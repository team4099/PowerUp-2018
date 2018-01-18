package org.usfirst.frc.team4099.lib.util.motion

class MotionProfileConstraints {

    protected var max_abs_vel_: Double = Double.POSITIVE_INFINITY
    protected var max_abs_acc_: Double = Double.POSITIVE_INFINITY

    constructor(max_vel: Double, max_acc: Double) {
        max_abs_vel_ = max_vel
        max_abs_acc_ = max_acc
    }

    fun max_abs_vel(): Double {
        return max_abs_vel_
    }

    fun max_abs_acc(): Double {
        return max_abs_acc_
    }

    override fun equals(other: Any?): Boolean {
        if(!(other is MotionProfileConstraints)) {
            return false
        }
        val MPC: MotionProfileConstraints = other as MotionProfileConstraints
        return (other.max_abs_acc_ == max_abs_acc_) && (other.max_abs_vel_ == max_abs_vel_)
    }
}