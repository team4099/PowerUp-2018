package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class LeftTo1 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,275.0,0.0,60.0));
        sWaypoints.add(Waypoint(215.0,275.0,20.0,60.0));
        sWaypoints.add(Waypoint(248.0,224.0,30.0,60.0));
        sWaypoints.add(Waypoint(207.0,224.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 275.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":275},"speed":60,"radius":0,"comment":""},{"position":{"x":215,"y":275},"speed":60,"radius":20,"comment":""},{"position":{"x":248,"y":224},"speed":60,"radius":30,"comment":""},{"position":{"x":207,"y":224},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftTo1
}