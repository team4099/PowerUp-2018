package org.usfirst.frc.team4099.auto.motionprofiling

import jaci.pathfinder.*

/**
 * Created by azhong2002 on 9/8/2018.
 * Generates path based on given waypoints.
 */

class PathGenerator {

    fun generatePath(waypoints : WayPoint[]) : modifier{
        val config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION, CONSTANTS.MEX_JERK)
        val path = Pathfinder.generate(waypoints, config)
        val modifier = TankModifier(trajectory).modify(Constants.WHEEL_BASE_WIDTH)
        return modifier
    }
}