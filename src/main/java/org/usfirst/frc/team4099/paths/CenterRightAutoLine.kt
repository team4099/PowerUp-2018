package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class CenterRightAutoLine : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(20.0,155.0,0.0,60.0));
        sWaypoints.add(Waypoint(64.0,155.0,40.0,60.0));
        sWaypoints.add(Waypoint(84.0,97.0,0.0,60.0));
        sWaypoints.add(Waypoint(98.0,64.0,30.0,60.0));
        sWaypoints.add(Waypoint(148.0,58.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(20.0, 155.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":20,"y":155},"speed":60,"radius":0,"comment":""},{"position":{"x":64,"y":155},"speed":60,"radius":40,"comment":""},{"position":{"x":84,"y":97},"speed":60,"radius":0,"comment":""},{"position":{"x":98,"y":64},"speed":60,"radius":30,"comment":""},{"position":{"x":148,"y":58},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: CenterRightAutoLine
}