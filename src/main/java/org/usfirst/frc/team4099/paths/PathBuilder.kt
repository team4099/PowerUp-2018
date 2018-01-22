package org.usfirst.frc.team4099.paths

//import org.usfirst.frc.team4099.auto.actions.WaitForPathMarkerAction
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.control.PathSegment
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D

import kotlin.collections.List

class PathBuilder {

    fun buildPathFromWaypoints(w: List<Waypoint>): Path {
        val p: Path = Path()
        if (w.size < 2) {
            throw Error("Path must contain at least 2 waypoints")
        }
        var i: Int = 0
        if (w.size > 2) {
            do {
                Arc(getPoint(w,i), getPoint(w, i+1), getPoint(w, i+2)).addToPath(p)
                i++
            } while (i < w.size - 2)
        }
        Line(w[w.size - 2], w[w.size - 1]).addToPath(p, 0.0)
        p.extrapolateLast()
        p.verifySpeeds()
        return p
    }

    private fun getPoint(w: List<Waypoint>, i: Int): Waypoint {
        if (i > w.size) {
            return w[w.size - 1]
        }
        return w[i]
    }

    companion object {
        private val kEpsilon: Double = 1E-9
        private val kReallyBigNumber: Double = 1E9

        private fun intersect(l1: Line, l2: Line): Translation2D {
            val lineA = RigidTransform2D(l1.end, Rotation2D(l1.slope, true).normal())
            val lineB = RigidTransform2D(l2.start, Rotation2D(l2.slope, true).normal())
            return lineA.intersection(lineB)
        }

        class Waypoint {

            var position: Translation2D
            var radius: Double
            var speed: Double
            var marker: String? = null

            constructor(x: Double, y: Double, r: Double, s: Double) {
                position = Translation2D(x, y)
                radius = r
                speed = s
            }

            constructor(pos: Translation2D, r: Double, s: Double) {
                position = pos
                radius = r
                speed = s
            }

            constructor(other: Waypoint) {
                position = other.position
                radius = other.radius
                speed = other.speed
                marker = other.marker
            }

            constructor(x: Double, y: Double, r: Double, s: Double, m: String) {
                position = Translation2D(x, y)
                radius = r
                speed = s
                marker = m
            }
        }

        class Line(wa: Waypoint, wb: Waypoint) {
            var a: Waypoint = wa
            var b: Waypoint = wb
            var start: Translation2D
            var end: Translation2D
            var slope: Translation2D
            var speed: Double

            internal fun addToPath(p: Path, endS: Double) {
                val pathLength : Double = Translation2D(end, start).norm()
                if (pathLength > kEpsilon) {
                    if (b.marker != null) {
                        p.addSegment(PathSegment(start.x(), start.y(), end.x(), end.y(), b.speed, p.getLastMotionState(), endS, b.marker!!))
                        p.addSegment(PathSegment(start.x(), start.y(), end.x(), end.y(), b.speed, p.getLastMotionState(), endS))
                    }
                }
            }

            init {
                slope = Translation2D(a.position, b.position)
                speed = b.speed
                start = a.position.translateBy(slope.scale(a.radius / slope.norm()))
                end = b.position.translateBy(slope.scale(-b.radius / slope.norm()))
            }
        }

        class Arc {
            var a: Line
            var b: Line
            var center: Translation2D
            var radius: Double
            var speed: Double

            constructor (wa: Waypoint, wb: Waypoint, wc: Waypoint) {
                a = Line(wa,wb)
                b = Line(wb,wc)
                speed = (a.speed + b.speed) / 2
                center = intersect(a,b)
                radius = Translation2D(center, a.end).norm()
            }

            constructor (la: Line, lb: Line) {
                a = la
                b = lb
                speed = (la.speed + lb.speed) / 2
                center = intersect(a,b)
                radius = Translation2D(center, a.end).norm()
            }

            internal fun addToPath(p: Path) {
                a.addToPath(p, speed)
                if (radius > kEpsilon && radius < kReallyBigNumber) {
                    p.addSegment(PathSegment(a.end.x(), a.end.y(), b.start.x(), b.start.y(), center.x(), center.y(), speed, p.getLastMotionState(), b.speed))
                }
            }
        }
    }
}