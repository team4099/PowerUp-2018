package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class RightTo24 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(20.0,47.0,0.0,60.0));
        sWaypoints.add(Waypoint(215.0,47.0,20.0,60.0));
        sWaypoints.add(Waypoint(273.0,101.0,60.0,60.0));
        sWaypoints.add(Waypoint(207.0,99.0,5.0,60.0));
        sWaypoints.add(Waypoint(229.0,99.0,0.0,60.0));
        sWaypoints.add(Waypoint(272.0,52.0,0.0,60.0));
        sWaypoints.add(Waypoint(286.0,38.0,10.0,60.0));
        sWaypoints.add(Waypoint(302.0,26.0,10.0,60.0));
        sWaypoints.add(Waypoint(324.0,32.0,10.0,60.0));
        sWaypoints.add(Waypoint(324.0,50.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(20.0, 47.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":20,"y":47},"speed":60,"radius":0,"comment":""},{"position":{"x":215,"y":47},"speed":60,"radius":20,"comment":""},{"position":{"x":273,"y":101},"speed":60,"radius":60,"comment":""},{"position":{"x":207,"y":99},"speed":60,"radius":5,"comment":""},{"position":{"x":229,"y":99},"speed":60,"radius":0,"comment":""},{"position":{"x":272,"y":52},"speed":60,"radius":0,"comment":""},{"position":{"x":286,"y":38},"speed":60,"radius":10,"comment":""},{"position":{"x":302,"y":26},"speed":60,"radius":10,"comment":""},{"position":{"x":324,"y":32},"speed":60,"radius":10,"comment":""},{"position":{"x":324,"y":50},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightTo24
}