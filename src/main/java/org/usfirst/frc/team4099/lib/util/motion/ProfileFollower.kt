package org.usfirst.frc.team4099.lib.util.motion

import org.usfirst.frc.team4099.lib.util.motion.MotionProfileGoal.CompletionBehavior

open class ProfileFollower {
    protected var mKp: Double = 0.0
    protected var mKi: Double = 0.0
    protected var mKv: Double = 0.0
    protected var mKffv: Double = 0.0
    protected var mKffa: Double = 0.0

    protected var mMinOutput: Double = Double.NEGATIVE_INFINITY
    protected var mMaxOutput: Double = Double.POSITIVE_INFINITY
    protected var mLatestActualState: MotionState = MotionState.kInvalidState
    protected var mInitialState: MotionState = MotionState.kInvalidState
    protected var mLatestPosError: Double = 0.0
    protected var mLatestVelError: Double = 0.0
    protected var mTotalError: Double = 0.0

    protected var mGoal: MotionProfileGoal? = null
    protected var mConstraints: MotionProfileConstraints? = null
    protected var mSetpointGenerator: SetpointGenerator = SetpointGenerator()
    protected var mLatestSetpoint: SetpointGenerator.Setpoint? = null

    constructor(kp: Double, ki: Double, kv: Double, kffv: Double, kffa: Double) {
        resetProfile()
        setGains(kp, ki, kv, kffv, kffa)
    }

    fun setGains(kp: Double, ki: Double, kv: Double, kffv: Double, kffa: Double) {
        mKp = kp
        mKi = ki
        mKv = kv
        mKffv = kffv
        mKffa = kffa
    }

    fun resetProfile() {
        mTotalError = 0.0
        mInitialState = MotionState.kInvalidState
        mLatestActualState = MotionState.kInvalidState
        mLatestPosError = Double.NaN
        mLatestVelError = Double.NaN
        mSetpointGenerator.reset()
        mGoal = null
        mConstraints = null
        resetSetpoint()
    }

    fun setGoalAndConstraints(goal: MotionProfileGoal, constraints: MotionProfileConstraints) {
        if (mGoal != null && mGoal != goal && mLatestSetpoint != null) {
            mLatestSetpoint!!.final_setpoint = false
        }
        mGoal = goal
        mConstraints = constraints
    }

    fun setGoal(goal: MotionProfileGoal) {
        mConstraints?.let { setGoalAndConstraints(goal, it) }
    }

    fun getGoal(): MotionProfileGoal? {
        return mGoal
    }

    fun setConstraints(constraints: MotionProfileConstraints) {
        mGoal?.let { setGoalAndConstraints(it, constraints) }
    }

    fun getSetpoint(): MotionState {
        if (mLatestSetpoint == null) {
            return MotionState.kInvalidState
        }
        return mLatestSetpoint!!.motion_state
    }

    fun resetSetpoint() {
        mLatestSetpoint = null
    }

    fun resetIntegral() {
        mTotalError = 0.0
    }

    @Synchronized open fun update(latest_state: MotionState, t: Double): Double {
        mLatestActualState = latest_state
        var prev_state: MotionState = latest_state
        if (mLatestSetpoint != null) {
            prev_state = mLatestSetpoint!!.motion_state
        } else {
            mInitialState = prev_state
        }
        val dt: Double = Math.max(0.0, t - prev_state.t())
        mLatestSetpoint = mSetpointGenerator.getSetpoint(mConstraints!!, mGoal!!, prev_state, t)

        mLatestPosError = mLatestSetpoint!!.motion_state.pos() - latest_state.pos()
        mLatestVelError = mLatestSetpoint!!.motion_state.vel() - latest_state.vel()
        var output: Double = mKp * mLatestPosError + mKv * mLatestVelError + mKffv * mLatestSetpoint!!.motion_state.vel()
        if (!mLatestSetpoint!!.motion_state.acc().isNaN()) {
            output += mKffa * mLatestSetpoint!!.motion_state.acc()
        }
        if (output >= mMinOutput && output <= mMaxOutput) {
            mTotalError += mLatestPosError * dt
            output += mKi * mTotalError
        } else {
            mTotalError = 0.0
        }

        output = Math.max(mMinOutput, Math.min(mMaxOutput, output))

        return output
    }

    fun setMinOutput(mino: Double) {
        mMinOutput = mino
    }

    fun setMaxOutput(maxo: Double) {
        mMaxOutput = maxo
    }

    fun getPosError(): Double {
        return mLatestPosError
    }

    fun getVelError(): Double {
        return mLatestVelError
    }

    fun isFinishedProfile(): Boolean {
        return mGoal != null && mLatestSetpoint != null && mLatestSetpoint!!.final_setpoint
    }

    fun onTarget(): Boolean {
        if (mGoal == null || mLatestSetpoint == null) {
            return false
        }

        val goal_to_start: Double = mGoal!!.pos() - mInitialState.pos()
        val goal_to_actual: Double = mGoal!!.pos() - mLatestActualState.pos()
        val passed_goal_state: Boolean = Math.signum(goal_to_start) * Math.signum(goal_to_actual) < 0.0
        return mGoal!!.atGoalState(mLatestActualState) || (mGoal!!.comp_behav() != CompletionBehavior.OVERSHOOT && passed_goal_state)
    }
}