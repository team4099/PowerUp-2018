package org.usfirst.frc.team4099.auto.actions

import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier
import org.usfirst.frc.team4099.auto.motionprofiling.AutoConstants
import org.usfirst.frc.team4099.robot.subsystems.Drive

class FollowPathAction(waypoints : Array<Waypoint>) : Action{
    private val mDrive: Drive = Drive.instance
    val config : Trajectory.Config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION, AutoConstants.MAX_JERK)
    val path : Trajectory = Pathfinder.generate(waypoints, config)
    val modifier : TankModifier = TankModifier(path).modify(AutoConstants.WHEEL_BASE_WIDTH)

    init{

    }
    override fun start(){
        ForwardDistanceAction(150.0)
        mDrive.arcadeDrive(0.4, 0.0)
        mDrive.enablePathFollow(modifier)
    }
    override fun update(){
        mDrive.arcadeDrive(0.4, 0.0)
        mDrive.updatePathFollower()
    }
    override fun done(){
        println("------- END PATHFOLLOW -------")
    }
    override fun isFinished() = false
}