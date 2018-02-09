﻿package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class RightTo2 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,47.0,0.0,60.0));
        sWaypoints.add(Waypoint(215.0,47.0,20.0,60.0));
        sWaypoints.add(Waypoint(277.0,99.0,60.0,60.0));
        sWaypoints.add(Waypoint(207.0,99.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 47.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":47},"speed":60,"radius":0,"comment":""},{"position":{"x":215,"y":47},"speed":60,"radius":20,"comment":""},{"position":{"x":277,"y":99},"speed":60,"radius":60,"comment":""},{"position":{"x":207,"y":99},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightTo2
}