package src.main.kotlin.org.usfirst.frc.team4099.auto.paths

class RightToBase : PathMaker{
    var points = arrayOf<Waypoint>(Waypoint(0, 0, 0), Waypoint(0, 10, 0))
    override fun getWayPoints(): arrayOf<Waypoint>{
        return points
    }
}