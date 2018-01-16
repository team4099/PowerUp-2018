package org.usfirst.frc.team4099.lib.util.motion

import org.usfirst.frc.team4099.lib.util.Utils.epsilonEquals
import javax.annotation.processing.Completion

class MotionProfileGoal {
    protected var pos_: Double = 0.0
    protected var max_abs_vel_: Double = 0.0
    protected var comp_behav_: CompletionBehavior = CompletionBehavior.OVERSHOOT
    protected var pos_toler_: Double = 1E-3
    protected var vel_toler_: Double = 1E-2

    @JvmOverloads constructor(pos: Double = 0.0, max_abs_vel: Double = 0.0, comp_behav: CompletionBehavior = CompletionBehavior.OVERSHOOT, pos_toler: Double = 1E-3, vel_toler: Double = 1E-2) {
        pos_ = pos
        max_abs_vel_ = max_abs_vel
        comp_behav_ = comp_behav
        pos_toler_ = pos_toler
        vel_toler_ = vel_toler
        sanityCheck()
    }

    constructor(other: MotionProfileGoal) {
        MotionProfileGoal(other.pos_, other.max_abs_vel_, other.comp_behav_, other.pos_toler_, other.vel_toler_)
    }

    fun flipped(): MotionProfileGoal {
        return MotionProfileGoal(-pos_, max_abs_vel_, comp_behav_, pos_toler_, vel_toler_)
    }

    fun pos(): Double {
        return pos_
    }

    fun max_abs_vel(): Double {
        return max_abs_vel_
    }

    fun comp_behav(): CompletionBehavior {
        return comp_behav_
    }

    fun pos_toler(): Double {
        return pos_toler_
    }

    fun vel_toler(): Double {
        return vel_toler_
    }

    fun atGoalState(state: MotionState): Boolean {
        return atGoalPos(state.pos()) && (Math.abs(state.vel()) < (max_abs_vel_ + vel_toler_) || comp_behav_ == CompletionBehavior.VIOLATE_MAX_ABS_VEL)
    }

    fun atGoalPos(pos: Double): Boolean {
        return epsilonEquals(pos, pos_, pos_toler_)
    }

    fun sanityCheck() {
        if (max_abs_vel_ > vel_toler_ && comp_behav_ == CompletionBehavior.OVERSHOOT) {
            comp_behav_ = CompletionBehavior.VIOLATE_MAX_ABS_VEL
        }
    }

    override fun toString(): String {
        return "pos: " + pos_ + " (+/- " + pos_toler_ + "), max_abs_vel: " + max_abs_vel_ + " (+/- " + vel_toler_ + "), completion behavior: " + comp_behav_.name
    }

    override fun equals(other: Any?): Boolean {
        if(!(other is MotionProfileGoal)) {
            return false
        }
        val mpg: MotionProfileGoal = other as MotionProfileGoal
        return (other.comp_behav_ == comp_behav_) && (other.pos_ == pos_) && (other.max_abs_vel_ == max_abs_vel_) && (other.pos_toler_ == pos_toler_) && (other.vel_toler_ == vel_toler_)
    }

    enum class CompletionBehavior {
        OVERSHOOT,
        VIOLATE_MAX_ACCEL,
        VIOLATE_MAX_ABS_VEL
    }
}