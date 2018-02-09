package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class LeftTo14 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,275.0,0.0,60.0));
        sWaypoints.add(Waypoint(223.0,270.0,20.0,60.0));
        sWaypoints.add(Waypoint(248.0,224.0,30.0,60.0));
        sWaypoints.add(Waypoint(207.0,224.0,0.0,60.0));
        sWaypoints.add(Waypoint(236.0,228.0,18.0,60.0));
        sWaypoints.add(Waypoint(241.0,72.0,40.0,60.0));
        sWaypoints.add(Waypoint(283.0,26.0,20.0,60.0));
        sWaypoints.add(Waypoint(324.0,35.0,10.0,60.0));
        sWaypoints.add(Waypoint(324.0,50.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 275.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":275},"speed":60,"radius":0,"comment":""},{"position":{"x":223,"y":270},"speed":60,"radius":20,"comment":""},{"position":{"x":248,"y":224},"speed":60,"radius":30,"comment":""},{"position":{"x":207,"y":224},"speed":60,"radius":0,"comment":""},{"position":{"x":236,"y":228},"speed":60,"radius":18,"comment":""},{"position":{"x":241,"y":72},"speed":60,"radius":40,"comment":""},{"position":{"x":283,"y":26},"speed":60,"radius":20,"comment":""},{"position":{"x":324,"y":35},"speed":60,"radius":10,"comment":""},{"position":{"x":324,"y":50},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftTo14
}