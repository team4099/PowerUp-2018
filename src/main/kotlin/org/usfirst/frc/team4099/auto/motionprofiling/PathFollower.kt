package org.usfirst.frc.team4099.auto.motionprofiling

import jaci.pathfinder.*

/**
 * Created by azhong2002 on 9/8/2018.
 */

class PathFollower{
    var left = EncoderFollower(modifier.getLeftTrajectory())
    var right = EncoderFollower(modifier.getRightTrajectory())

}