package org.usfirst.frc.team4099.lib.util.control

import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Twist2D
import org.usfirst.frc.team4099.lib.util.motion.MotionProfileConstraints
import org.usfirst.frc.team4099.lib.util.motion.MotionProfileGoal
import org.usfirst.frc.team4099.lib.util.motion.MotionProfileGoal.CompletionBehavior
import org.usfirst.frc.team4099.lib.util.motion.MotionState
import org.usfirst.frc.team4099.lib.util.motion.ProfileFollower

/**
 * A PathFollower follows a predefined path using a combination of feedforward and feedback control. It uses an
 * AdaptivePurePursuitController to choose a reference pose and generate a steering command (curvature), and then a
 * ProfileFollower to generate a profile (displacement and velocity) command.
 */

class PathFollower{

    companion object {

        private val kReallyBigNumber: Double = 1E6

        class DebugOutput{
            var t: Double = 0.0
            var pose_x: Double = 0.0
            var pose_y: Double =0.0
            var pose_theta: Double = 0.0
            var linear_displacement: Double =0.0
            var linear_velocity: Double = 0.0
            var profile_displacement: Double = 0.0
            var profile_velocity: Double = 0.0
            var velocity_command_dx: Double = 0.0
            var velocity_command_dy: Double = 0.0
            var velocity_command_dtheta: Double = 0.0
            var steering_command_dx: Double = 0.0
            var steering_command_dy: Double = 0.0
            var steering_command_dtheta: Double = 0.0
            var cross_track_error: Double = 0.0
            var along_track_error: Double = 0.0
            var lookahead_point_x: Double = 0.0
            var lookahead_point_y: Double = 0.0
            var lookahead_point_velocity: Double = 0.0
        }
        class Parameters{
            val lookahead: Lookahead
            val inertia_gain: Double
            val profile_kp: Double
            val profile_ki: Double
            val profile_kv: Double
            val profile_kffv: Double
            val profile_kffa: Double
            val profile_max_abs_vel: Double
            val profile_max_abs_acc: Double
            val goal_pos_tolerance: Double
            val goal_vel_tolerance: Double
            val stop_steering_distance: Double

            constructor(lookahead: Lookahead, inertia_gain: Double, profile_kp: Double, profile_ki: Double, profile_kv: Double,
            profile_kffv: Double, profile_kffa: Double, profile_max_abs_vel: Double, profile_max_abs_acc: Double,
            goal_pos_tolerance: Double, goal_vel_tolerance: Double, stop_steering_distance: Double){
                this.lookahead = lookahead
                this.inertia_gain = inertia_gain
                this.profile_kp = profile_kp
                this.profile_ki = profile_ki
                this.profile_kv = profile_kv
                this.profile_kffv = profile_kffv
                this.profile_kffa = profile_kffa
                this.profile_max_abs_vel = profile_max_abs_vel
                this.profile_max_abs_acc = profile_max_abs_acc
                this.goal_pos_tolerance = goal_pos_tolerance
                this.goal_vel_tolerance = goal_vel_tolerance
                this.stop_steering_distance = stop_steering_distance
            }
        }
    }

    var mSteeringController: AdaptivePurePursuitController
    var mLastSteeringDelta: Twist2D
    var mVelocityController: ProfileFollower
    val mInertiaGain: Double
    var overrideFinished: Boolean = false
    var doneSteering: Boolean = false
    var mDebugOutput: DebugOutput = DebugOutput()

    var mMaxProfileVel: Double
    var mMaxProfileAcc: Double
    val mGoalPosTolerance: Double
    val mGoalVelTolerance: Double
    val mStopSteeringDistance: Double
    var mCrossTrackError: Double = 0.0
    var mAlongTrackError: Double = 0.0

    constructor(path: Path, reversed: Boolean, parameters: Parameters){
        mSteeringController = AdaptivePurePursuitController(path, reversed, parameters.lookahead)
        mLastSteeringDelta = Twist2D.identity()
        mVelocityController = ProfileFollower(parameters.profile_kp, parameters.profile_ki, parameters.profile_kv,
                parameters.profile_kffv, parameters.profile_kffa)
        mVelocityController.setConstraints(MotionProfileConstraints(parameters.profile_max_abs_vel, parameters.profile_max_abs_acc))
        mMaxProfileVel = parameters.profile_max_abs_vel
        mMaxProfileAcc = parameters.profile_max_abs_acc
        mGoalPosTolerance = parameters.goal_pos_tolerance
        mGoalVelTolerance = parameters.goal_vel_tolerance
        mInertiaGain = parameters.inertia_gain
        mStopSteeringDistance = parameters.stop_steering_distance
    }
    @Synchronized fun update(t: Double, pose: RigidTransform2D, displacement: Double, velocity: Double): Twist2D{
        if (!mSteeringController.isFinished()){
            val steering_command: AdaptivePurePursuitController.Companion.Command = mSteeringController.update(pose)
            mDebugOutput.lookahead_point_x = steering_command.lookahead_point.x()
            mDebugOutput.lookahead_point_y = steering_command.lookahead_point.y()
            mDebugOutput.lookahead_point_velocity = steering_command.end_velocity
            mDebugOutput.steering_command_dx = steering_command.delta.dx()
            mDebugOutput.steering_command_dy = steering_command.delta.dy()
            mDebugOutput.steering_command_dtheta = steering_command.delta.dtheta()
            mCrossTrackError = steering_command.cross_track_error
            mLastSteeringDelta = steering_command.delta
            mVelocityController.setGoalAndConstraints(MotionProfileGoal(displacement+steering_command.delta.dx(),
                        Math.abs(steering_command.end_velocity),CompletionBehavior.VIOLATE_MAX_ABS_VEL,
                        mGoalPosTolerance, mGoalVelTolerance),
                    MotionProfileConstraints(Math.min(mMaxProfileVel,steering_command.max_velocity), mMaxProfileAcc))
            if (steering_command.remaining_path_length < mStopSteeringDistance){
                    doneSteering = true
            }
        }

        val velocity_command: Double = mVelocityController.update(MotionState(t, displacement,velocity,0.0), t)
        mAlongTrackError = mVelocityController.getPosError()
        val curvature: Double = mLastSteeringDelta.dtheta() / mLastSteeringDelta.dx()
        var dtheta: Double = mLastSteeringDelta.dtheta()
        if(!curvature.isNaN() && Math.abs(curvature)< kReallyBigNumber){
            // Regenerate angular velocity command from adjusted curvature
            val abs_velocity_setpoint: Double = Math.abs(mVelocityController.getSetpoint().vel())
            dtheta = mLastSteeringDelta.dx() * curvature * (1.0 + mInertiaGain * abs_velocity_setpoint)
        }
        val scale: Double = velocity_command / mLastSteeringDelta.dx()
        val rv: Twist2D = Twist2D(mLastSteeringDelta.dx()*scale, 0.0, dtheta * scale)

        // Fill out debug
        mDebugOutput.t = t
        mDebugOutput.pose_x = pose.getTranslation().x()
        mDebugOutput.pose_y = pose.getTranslation().y()
        mDebugOutput.pose_theta = pose.getRotation().radians
        mDebugOutput.linear_displacement = displacement
        mDebugOutput.linear_velocity = velocity
        mDebugOutput.profile_displacement = mVelocityController.getSetpoint().pos()
        mDebugOutput.profile_velocity = mVelocityController.getSetpoint().vel()
        mDebugOutput.velocity_command_dx = rv.dx()
        mDebugOutput.velocity_command_dy = rv.dy()
        mDebugOutput.velocity_command_dtheta = rv.dtheta()
        mDebugOutput.cross_track_error = mCrossTrackError
        mDebugOutput.along_track_error = mAlongTrackError

        return rv
    }

    fun getCrossTrackError():Double{
        return mCrossTrackError
    }
    fun getAlongTrackError(): Double{
        return mAlongTrackError
    }
    fun getDebug(): DebugOutput{
        return mDebugOutput
    }
    fun isFinished(): Boolean{
        return (mSteeringController.isFinished() && mVelocityController.isFinishedProfile() &&
                mVelocityController.onTarget() || overrideFinished)
    }
    fun forceFinish(){
        overrideFinished = true
    }
    fun hasPassedMarker(marker: String): Boolean{
        return mSteeringController.hasPassedMarker(marker)
    }

}
