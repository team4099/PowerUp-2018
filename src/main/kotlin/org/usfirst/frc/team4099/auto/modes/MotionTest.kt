package src.main.kotlin.org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.modes.AutoModeBase
import org.usfirst.frc.team4099.auto.motionprofiling.AutoConstants
import jaci.pathfinder.*
import org.usfirst.frc.team4099.robot.subsystems.Drive
import jaci.pathfinder.*
import jaci.pathfinder.modifiers.*
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.followers.*
import jaci.pathfinder.Waypoint
class MotionTest(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase(){
    private val mDrive: Drive = Drive.instance
    var points = arrayOf<Waypoint>(Waypoint(0.0, 0.0, 0.0), // Waypoint @ x=-4, y=-1, exit angle=0 degrees
            Waypoint(4.0, 0.0, 0.0), // Waypoint @ x=-2, y=-2, exit angle=0 radians
            Waypoint(4.0, 4.0, Pathfinder.d2r(90.0))                           // Waypoint @ x=0, y=0,   exit angle=45 radians
    )



    override fun routine() {
        println("begin routine")
        val config : Trajectory.Config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION, AutoConstants.MAX_JERK)
        val path : Trajectory = Pathfinder.generate(points, config)
        val modifier : TankModifier = TankModifier(path).modify(AutoConstants.WHEEL_BASE_WIDTH)
        mDrive.currentState = Drive.DriveControlState.PATH_FOLLOWING
        //pathGenerator = PathGenerator()
        mDrive.path = Pathfinder.generate(points, config)
        mDrive.leftEncoderFollower =  EncoderFollower(modifier.leftTrajectory)
        mDrive.rightEncoderFollower =  EncoderFollower(modifier.rightTrajectory)
        //mDrive.enablePathFollow(points, modifier)
        println("finish routine")
    }

 /*   fun generatePath(waypoints : WayPoint[]) : TankModifier{
        val config : Trajectory.config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION, AutoConstants.MAX_JERK)
        val path : Trajectory = Pathfinder.generate(waypoints, config)
        val modifier : TankModifer = TankModifier(trajectory).modify(AutoConstants.WHEEL_BASE_WIDTH)
        return modifier
    }*/

}