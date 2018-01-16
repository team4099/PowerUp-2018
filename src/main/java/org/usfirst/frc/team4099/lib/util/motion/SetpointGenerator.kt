package org.usfirst.frc.team4099.lib.util.motion

import java.util.Optional

class SetpointGenerator() {
    data class Setpoint(var motion_state: MotionState, var final_setpoint: Boolean)

    protected var mProfile: MotionProfile? = null
    protected var mGoal: MotionProfileGoal? = null
    protected var mConstraints: MotionProfileConstraints? = null

    fun reset() {
        mProfile = null
        mGoal = null
        mConstraints = null
    }

    @Synchronized fun getSetpoint(constraints: MotionProfileConstraints, goal: MotionProfileGoal, prev_state: MotionState, t: Double): Setpoint {
        var regenerate: Boolean = mConstraints == null || mConstraints != constraints || mGoal == null || mGoal != goal || mProfile == null
        if(!regenerate && !mProfile!!.isEmpty()) {
            var expected_state: Optional<MotionState> = mProfile!!.stateByTime(prev_state.t())
            regenerate = !expected_state.isPresent() || !expected_state.get().equals(prev_state)
        }

        if(regenerate) {
            mConstraints = constraints
            mGoal = goal
            mProfile = MotionProfileGenerator.generateProfile(constraints, goal, prev_state)
        }

        var rv: Setpoint? = null
        if(!mProfile!!.isEmpty() && mProfile!!.isValid()) {
            var setpoint: MotionState
            if (t > mProfile!!.endTime()) {
                setpoint = mProfile!!.endState()
            } else if (t <mProfile!!.startTime()) {
                setpoint = mProfile!!.startState()
            } else {
                setpoint = mProfile!!.stateByTime(t).get()
            }

            mProfile!!.trimBeforeTime(t)
            rv = Setpoint(setpoint, mProfile!!.isEmpty() || mGoal!!.atGoalState(setpoint))
        }
        if (rv == null){
            rv = Setpoint(prev_state, true)
        }

        if (rv.final_setpoint) {
            rv.motion_state = MotionState(rv.motion_state.t(), mGoal!!.pos(), Math.signum(rv.motion_state.vel()) * Math.max(mGoal!!.max_abs_vel(), Math.abs(rv.motion_state.vel())), 0.0)
        }

        return rv
    }

    fun getProfile(): MotionProfile? {
        return mProfile
    }
}