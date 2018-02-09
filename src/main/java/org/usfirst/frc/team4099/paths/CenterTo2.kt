package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class CenterTo2 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,155.0,0.0,60.0));
        sWaypoints.add(Waypoint(64.0,155.0,40.0,60.0));
        sWaypoints.add(Waypoint(78.0,116.0,0.0,60.0));
        sWaypoints.add(Waypoint(102.0,68.0,30.0,60.0));
        sWaypoints.add(Waypoint(142.0,62.0,0.0,60.0));
        sWaypoints.add(Waypoint(192.0,60.0,0.0,60.0));
        sWaypoints.add(Waypoint(217.0,61.0,10.0,60.0));
        sWaypoints.add(Waypoint(236.0,74.0,15.0,60.0));
        sWaypoints.add(Waypoint(229.0,103.0,15.0,60.0));
        sWaypoints.add(Waypoint(207.0,103.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 155.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":155},"speed":60,"radius":0,"comment":""},{"position":{"x":64,"y":155},"speed":60,"radius":40,"comment":""},{"position":{"x":78,"y":116},"speed":60,"radius":0,"comment":""},{"position":{"x":102,"y":68},"speed":60,"radius":30,"comment":""},{"position":{"x":142,"y":62},"speed":60,"radius":0,"comment":""},{"position":{"x":192,"y":60},"speed":60,"radius":0,"comment":""},{"position":{"x":217,"y":61},"speed":60,"radius":10,"comment":""},{"position":{"x":236,"y":74},"speed":60,"radius":15,"comment":""},{"position":{"x":229,"y":103},"speed":60,"radius":15,"comment":""},{"position":{"x":207,"y":103},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: CenterTo2
}