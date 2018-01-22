package org.usfirst.frc.team4099.paths.profiles

import org.usfirst.frc.team4099.paths.PathBuilder
import org.usfirst.frc.team4099.paths.PathBuilder.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

/**
 * Created by O on 1/22/2018.
 */

class PathAdapter{
    companion object {
        val kRobotProfile: RobotProfile = //our robot here
        val kFieldProfile: FieldProfile = // our field here

        // path variables
        val kLargeRadius: Double = 45.0
        val kModerateRadius: Double = 30.0
        val kNominalRadius: Double = 20.0
        val kSmallRadius: Double = 10.0
        val kSpeed: Double = 80.0

        // field and bot specific constants for the game
        // @see team 254's code for the type of constants to have
    }


}
