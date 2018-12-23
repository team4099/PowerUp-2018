package src.main.kotlin.org.usfirst.frc.team4099.auto.modes

import edu.wpi.first.wpilibj.DriverStation
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
import org.usfirst.frc.team4099.auto.actions.FollowPathAction
import org.usfirst.frc.team4099.auto.actions.ForwardDistanceAction

class MotionTest(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase(){

    var points = arrayOf(Waypoint(0.0, 0.0, 0.0), // Waypoint @ x=-4, y=-1, exit angle=0 degrees
            Waypoint(0.0, 0.0, 0.0), // Waypoint @ x=-2, y=-2, exit angle=0 radians
            Waypoint(0.0, 0.0, Pathfinder.d2r(0.0))                           // Waypoint @ x=0, y=0,   exit angle=45 radians
    )


    override fun routine() {
        //runAction(ForwardDistanceAction(150.0))
        runAction(FollowPathAction(points))
    }

    /*   fun generatePath(waypoints : WayPoint[]) : TankModifier{
           val config : Trajectory.config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION, AutoConstants.MAX_JERK)
           val path : Trajectory = Pathfinder.generate(waypoints, config)
           val modifier : TankModifer = TankModifier(trajectory).modify(AutoConstants.WHEEL_BASE_WIDTH)
           return modifier
       }*/

}