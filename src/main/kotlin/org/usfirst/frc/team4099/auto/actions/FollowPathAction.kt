package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.DriverStation
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier
import org.usfirst.frc.team4099.auto.motionprofiling.AutoConstants
import org.usfirst.frc.team4099.robot.subsystems.Drive

class FollowPathAction(waypoints : Array<Waypoint>) : Action{
    private val mDrive: Drive = Drive.instance
    val config : Trajectory.Config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION, AutoConstants.MAX_JERK)
    var points = arrayOf(Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)), // Waypoint @ x=-4, y=-1, exit angle=0 degrees
            Waypoint(10.0, 0.0, Pathfinder.d2r(45.0)) // Waypoint @ x=-2, y=-2, exit angle=0 radians
                           // Waypoint @ x=0, y=0,   exit angle=45 radians
    )
    var path : Trajectory = Pathfinder.generate(points, config)
    var modifier : TankModifier = TankModifier(path).modify(AutoConstants.WHEEL_BASE_WIDTH)

    init{
        DriverStation.reportError("Start Pathfollow", false)
    }
    override fun start(){
        //mDrive.arcadeDrive(0.4, 0.0)
        mDrive.enablePathFollow(modifier)

    }
    override fun update(){
        println("-----------------UPDATE PATHFOLLOW -----------------------")
        DriverStation.reportError("UPDATE PATHFOLLOW in Action", false)
        //mDrive.arcadeDrive(0.4, 0.0)
        mDrive.updatePathFollower()
    }override fun done(){
        println("------- END PATHFOLLOW -------")
    }
    override fun isFinished() = false
}