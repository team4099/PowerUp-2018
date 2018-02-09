package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class CenterTo24 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,155.0,0.0,60.0));
        sWaypoints.add(Waypoint(64.0,155.0,40.0,60.0));
        sWaypoints.add(Waypoint(78.0,116.0,0.0,60.0));
        sWaypoints.add(Waypoint(102.0,68.0,30.0,60.0));
        sWaypoints.add(Waypoint(142.0,62.0,0.0,60.0));
        sWaypoints.add(Waypoint(192.0,60.0,0.0,60.0));
        sWaypoints.add(Waypoint(217.0,61.0,10.0,60.0));
        sWaypoints.add(Waypoint(232.0,76.0,0.0,60.0));
        sWaypoints.add(Waypoint(229.0,99.0,10.0,60.0));
        sWaypoints.add(Waypoint(207.0,99.0,0.0,60.0));
        sWaypoints.add(Waypoint(253.0,58.0,0.0,60.0));
        sWaypoints.add(Waypoint(270.0,29.0,25.0,60.0));
        sWaypoints.add(Waypoint(300.0,25.0,15.0,60.0));
        sWaypoints.add(Waypoint(324.0,34.0,7.0,60.0));
        sWaypoints.add(Waypoint(324.0,50.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 155.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":155},"speed":60,"radius":0,"comment":""},{"position":{"x":64,"y":155},"speed":60,"radius":40,"comment":""},{"position":{"x":78,"y":116},"speed":60,"radius":0,"comment":""},{"position":{"x":102,"y":68},"speed":60,"radius":30,"comment":""},{"position":{"x":142,"y":62},"speed":60,"radius":0,"comment":""},{"position":{"x":192,"y":60},"speed":60,"radius":0,"comment":""},{"position":{"x":217,"y":61},"speed":60,"radius":10,"comment":""},{"position":{"x":232,"y":76},"speed":60,"radius":0,"comment":""},{"position":{"x":229,"y":99},"speed":60,"radius":10,"comment":""},{"position":{"x":207,"y":99},"speed":60,"radius":0,"comment":""},{"position":{"x":253,"y":58},"speed":60,"radius":0,"comment":""},{"position":{"x":270,"y":29},"speed":60,"radius":25,"comment":""},{"position":{"x":300,"y":25},"speed":60,"radius":15,"comment":""},{"position":{"x":324,"y":34},"speed":60,"radius":7,"comment":""},{"position":{"x":324,"y":50},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: CenterTo24
}