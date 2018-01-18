package org.usfirst.frc.team4099.lib.util.motion

import org.usfirst.frc.team4099.lib.util.math.Rotation2D


class HeadingProfileFollower: ProfileFollower{

    constructor(kp: Double, ki: Double, kv: Double, kffv: Double, kffa: Double) : super(kp, ki, kv, kffv, kffa)

    override fun update(latest_state: MotionState, t: Double): Double {
        val goal_rotation_inverse: Rotation2D = Rotation2D.fromDegrees(mGoal!!.pos()).inverse()
        if (mLatestSetpoint != null) {
            mLatestSetpoint!!.motion_state= MotionState(mLatestSetpoint!!.motion_state.t(), mGoal!!.pos() + goal_rotation_inverse.rotateBy(Rotation2D.fromDegrees(mLatestSetpoint!!.motion_state.pos())).degrees, mLatestSetpoint!!.motion_state.vel(), mLatestSetpoint!!.motion_state.acc())
        }
        val latest_state_unwrapped: MotionState = MotionState(latest_state.t(), mGoal!!.pos() + goal_rotation_inverse.rotateBy(Rotation2D.fromDegrees(latest_state.pos())).degrees, latest_state.vel(), latest_state.acc())
        var result: Double = super.update(latest_state_unwrapped, t)
        if (Math.abs(latest_state_unwrapped.pos() - mGoal!!.pos()) < mGoal!!.pos_toler()) {
            result = 0.0
            super.resetIntegral()
        }
        return result
    }

    companion object {
        fun canonicalize(state: MotionState): MotionState {
            return MotionState(state.t(), Rotation2D.fromDegrees(state.pos()).degrees, state.vel(), state.acc())
        }
    }
}
