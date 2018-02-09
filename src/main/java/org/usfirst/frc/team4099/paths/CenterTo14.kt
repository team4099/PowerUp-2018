package org.usfirst.frc.team4099.paths

import java.util.ArrayList

import org.usfirst.frc.team4099.paths.PathBuilder.Companion.Waypoint
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

class CenterTo14 : PathContainer {
    
    override fun buildPath() : Path {
        var sWaypoints : ArrayList<Waypoint> = ArrayList<Waypoint>()
        sWaypoints.add(Waypoint(21.0,155.0,0.0,60.0));
        sWaypoints.add(Waypoint(76.0,156.0,50.0,60.0));
        sWaypoints.add(Waypoint(96.0,264.0,50.0,60.0));
        sWaypoints.add(Waypoint(153.0,266.0,0.0,60.0));
        sWaypoints.add(Waypoint(221.0,263.0,20.0,60.0));
        sWaypoints.add(Waypoint(242.0,224.0,20.0,60.0));
        sWaypoints.add(Waypoint(207.0,224.0,0.0,60.0));
        sWaypoints.add(Waypoint(239.0,195.0,30.0,60.0));
        sWaypoints.add(Waypoint(238.0,74.0,20.0,60.0));
        sWaypoints.add(Waypoint(266.0,36.0,20.0,60.0));
        sWaypoints.add(Waypoint(324.0,28.0,15.0,60.0));
        sWaypoints.add(Waypoint(324.0,50.0,0.0,60.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints)
    }
    
    override fun getStartPose() : RigidTransform2D = RigidTransform2D(Translation2D(21.0, 155.0), Rotation2D.fromDegrees(180.0))

    override fun isReversed() : Boolean = false
	// WAYPOINT_DATA: [{"position":{"x":21,"y":155},"speed":60,"radius":0,"comment":""},{"position":{"x":76,"y":156},"speed":60,"radius":50,"comment":""},{"position":{"x":96,"y":264},"speed":60,"radius":50,"comment":""},{"position":{"x":153,"y":266},"speed":60,"radius":0,"comment":""},{"position":{"x":221,"y":263},"speed":60,"radius":20,"comment":""},{"position":{"x":242,"y":224},"speed":60,"radius":20,"comment":""},{"position":{"x":207,"y":224},"speed":60,"radius":0,"comment":""},{"position":{"x":239,"y":195},"speed":60,"radius":30,"comment":""},{"position":{"x":238,"y":74},"speed":60,"radius":20,"comment":""},{"position":{"x":266,"y":36},"speed":60,"radius":20,"comment":""},{"position":{"x":324,"y":28},"speed":60,"radius":15,"comment":""},{"position":{"x":324,"y":50},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: CenterTo14
}