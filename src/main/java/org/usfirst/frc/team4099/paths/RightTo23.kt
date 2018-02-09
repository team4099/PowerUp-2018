package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class RightTo23 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,47.0,0.0,0.0));
        sWaypoints.add(Waypoint(240.0,52.0,30.0,60.0));
        sWaypoints.add(Waypoint(235.0,99.0,18.0,60.0));
        sWaypoints.add(Waypoint(207.0,99.0,0.0,60.0));
        sWaypoints.add(Waypoint(235.0,99.0,25.0,60.0));
        sWaypoints.add(Waypoint(242.0,268.0,20.0,60.0));
        sWaypoints.add(Waypoint(287.0,304.0,20.0,60.0));
        sWaypoints.add(Waypoint(324.0,290.0,10.0,60.0));
        sWaypoints.add(Waypoint(324.0,272.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 47.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":47},"speed":0,"radius":0,"comment":""},{"position":{"x":240,"y":52},"speed":60,"radius":30,"comment":""},{"position":{"x":235,"y":99},"speed":60,"radius":18,"comment":""},{"position":{"x":207,"y":99},"speed":60,"radius":0,"comment":""},{"position":{"x":235,"y":99},"speed":60,"radius":25,"comment":""},{"position":{"x":242,"y":268},"speed":60,"radius":20,"comment":""},{"position":{"x":287,"y":304},"speed":60,"radius":20,"comment":""},{"position":{"x":324,"y":290},"speed":60,"radius":10,"comment":""},{"position":{"x":324,"y":272},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightTo23
}