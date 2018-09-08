package org.usfirst.frc.team4099.auto.motionprofiling

import jaci.pathfinder.*

/**
 * Created by azhong2002 on 9/8/2018.
 * Generates path based on given waypoints.
 */

class PathGenerator {

    fun generatePath(waypoints : WayPoint[]) : TankModifier{
        val config : Trajectory.config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION, AutoConstants.MAX_JERK)
        val path : Trajectory = Pathfinder.generate(waypoints, config)
        val modifier : TankModifer = TankModifier(trajectory).modify(AutoConstants.WHEEL_BASE_WIDTH)
        return modifier
    }

}