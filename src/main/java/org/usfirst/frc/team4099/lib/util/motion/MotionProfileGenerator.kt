package org.usfirst.frc.team4099.lib.util.motion

/**
 * Created by O on 1/12/2018.
 */

class MotionProfileGenerator{
    constructor(){
    }
    protected fun generateFlippedProfile(constraints: MotionProfileConstraints,
                                         goal_state: MotionProfileGoal,
                                         prev_state: MotionState):MotionProfile {
        var profile : MotionProfile = generateProfile(constraints, goal_state.flipped(), prev_state.flipped())
        for (s in profile.segments()){
            s.setStart(s.start().flipped())
            s.setEnd(s.end().flipped())
        }
        return profile
    }

    companion object {
        @Synchronized fun generateProfile(constraints: MotionProfileConstraints,
                                          goal_state: MotionProfileGoal,
                                          prev_state: MotionState):MotionProfile{
            var delta_pos = goal_state.pos()- prev_state.pos()
            if(delta_pos < 0.0 || (delta_pos == 0.0 && prev_state.vel() < 0.0)){
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
            if (start_state.pos()<0.0 && delta_pos > 0.0){
                val stopping_time = Math.abs(start_state.vel() / constraints.max_abs_acc())
                profile.appendControl(constraints.max_abs_acc(), stopping_time)
                start_state = profile.endState()
                delta_pos = goal_state.pos() - start_state.pos()
            }
            // invariant from now on: start_state.vel() >= 0.0
            val min_abs_vel_at_goal_sqr =  start_state.




        return profile
        }
    }

}