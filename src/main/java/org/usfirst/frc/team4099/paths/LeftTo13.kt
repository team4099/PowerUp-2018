package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class LeftTo13 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,275.0,0.0,60.0));
        sWaypoints.add(Waypoint(225.0,263.0,20.0,60.0));
        sWaypoints.add(Waypoint(242.0,224.0,20.0,60.0));
        sWaypoints.add(Waypoint(207.0,224.0,0.0,60.0));
        sWaypoints.add(Waypoint(256.0,279.0,10.0,60.0));
        sWaypoints.add(Waypoint(283.0,297.0,15.0,60.0));
        sWaypoints.add(Waypoint(324.0,298.0,20.0,60.0));
        sWaypoints.add(Waypoint(324.0,272.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 275.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":275},"speed":60,"radius":0,"comment":""},{"position":{"x":225,"y":263},"speed":60,"radius":20,"comment":""},{"position":{"x":242,"y":224},"speed":60,"radius":20,"comment":""},{"position":{"x":207,"y":224},"speed":60,"radius":0,"comment":""},{"position":{"x":256,"y":279},"speed":60,"radius":10,"comment":""},{"position":{"x":283,"y":297},"speed":60,"radius":15,"comment":""},{"position":{"x":324,"y":298},"speed":60,"radius":20,"comment":""},{"position":{"x":324,"y":272},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftTo13
}