package org.usfirst.frc.team4099.lib.util.motion

import org.usfirst.frc.team4099.lib.util.motion.MotionProfileGoal.CompletionBehavior

/**
 * Created by O on 1/12/2018.
 */

class MotionProfileGenerator {


    companion object {
        @Synchronized
        protected fun generateFlippedProfile(constraints: MotionProfileConstraints, goal_state: MotionProfileGoal, prev_state: MotionState): MotionProfile {
            var profile: MotionProfile = generateProfile(constraints, goal_state.flipped(), prev_state.flipped())
            for (s: MotionSegment in profile.segments()) {
                s.setStart(s.start().flipped())
                s.setEnd(s.end().flipped())
            }
            return profile
        }
        fun generateProfile(constraints: MotionProfileConstraints,
                            goal_state: MotionProfileGoal,
                            prev_state: MotionState): MotionProfile {
            var delta_pos = goal_state.pos() - prev_state.pos()
            if (delta_pos < 0.0 || (delta_pos == 0.0 && prev_state.vel() < 0.0)) {
                // if negative, flip to solve then flip the solution
                return generateFlippedProfile(constraints, goal_state, prev_state)
            }
            // From now on, delta_pos >= 0.0
            // Clamp the start state to be valid

            var start_state = MotionState(prev_state.t(), prev_state.pos(),
                    Math.signum(prev_state.vel()) * Math.min(Math.abs(prev_state.vel()), constraints.max_abs_vel()),
                    Math.signum(prev_state.acc()) * Math.min(Math.abs(prev_state.acc()), constraints.max_abs_acc()))

            var profile = MotionProfile()
            profile.reset(start_state)

            // if our velocity is headed away from the goal, the first thing we need to do is stop
            if (start_state.pos() < 0.0 && delta_pos > 0.0) {
                val stopping_time = Math.abs(start_state.vel() / constraints.max_abs_acc())
                profile.appendControl(constraints.max_abs_acc(), stopping_time)
                start_state = profile.endState()
                delta_pos = goal_state.pos() - start_state.pos()
            }
            // invariant from now on: start_state.vel() >= 0.0
            val min_abs_vel_at_goal_sqr: Double = start_state.vel2() - 2.0 * constraints.max_abs_acc() * delta_pos
            val min_abs_vel_at_goal: Double = Math.sqrt(Math.abs(min_abs_vel_at_goal_sqr))
            val max_abs_vel_at_goal: Double = Math.sqrt(start_state.vel2() + 2.0 * constraints.max_abs_acc() * delta_pos)
            var goal_vel: Double = goal_state.max_abs_vel()
            var max_acc: Double = constraints.max_abs_acc()
            if (min_abs_vel_at_goal_sqr > 0.0 && min_abs_vel_at_goal > (goal_state.max_abs_vel() + goal_state.vel_toler())) {
                if (goal_state.comp_behav() == CompletionBehavior.VIOLATE_MAX_ABS_VEL) {
                    goal_vel = min_abs_vel_at_goal
                } else if (goal_state.comp_behav() == CompletionBehavior.VIOLATE_MAX_ACCEL) {
                    if (Math.abs(delta_pos) < goal_state.pos_toler()) {
                        profile.appendSegment(MotionSegment(MotionState(profile.endTime(), profile.endPos(), profile.endState().vel(), Double.NEGATIVE_INFINITY), MotionState(profile.endTime(), profile.endPos(), goal_vel, Double.NEGATIVE_INFINITY)))
                        profile.consolidate()
                        return profile
                    }
                    max_acc = Math.abs(goal_vel * goal_vel - start_state.vel2() / 2.0 / delta_pos)
                } else {
                    val stop_t: Double = Math.abs(start_state.vel() / constraints.max_abs_acc())
                    profile.appendControl(-constraints.max_abs_acc(), stop_t)
                    profile.appendProfile(generateFlippedProfile(constraints, goal_state, profile.endState()))
                    profile.consolidate()
                    return profile
                }
            }

            goal_vel = Math.min(goal_vel, max_abs_vel_at_goal)
            val v_max: Double = Math.min(constraints.max_abs_vel(), Math.sqrt((start_state.vel2() + goal_vel * goal_vel) / 2.0 + delta_pos * max_acc))
            if (v_max > start_state.vel()) {
                val a_t: Double = (v_max - start_state.vel()) / max_acc
                profile.appendControl(max_acc, a_t)
                start_state = profile.endState()
            }

            val d_dcel: Double = Math.max(0.0, (start_state.vel2() - goal_vel * goal_vel) / 2.0 / constraints.max_abs_acc())
            val d_cruise: Double = Math.max(0.0, goal_state.pos() - start_state.pos() - d_dcel)
            if (d_cruise > 0.0) {
                val cruise_t: Double = d_cruise / start_state.vel()
                profile.appendControl(0.0, cruise_t)
                start_state = profile.endState()
            }

            if (d_dcel > 0.0) {
                val dcel_t: Double = (start_state.vel() - goal_vel) / max_acc
                profile.appendControl(-max_acc, dcel_t)
            }

            profile.consolidate()
            return profile
        }

    }
}

