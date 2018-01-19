package org.usfirst.frc.team4099.lib.util.control

import org.usfirst.frc.team4099.lib.util.math.Translation2D
import org.usfirst.frc.team4099.lib.util.motion.MotionState
import org.usfirst.frc.team4099.robot.Constants

import java.util.ArrayList
import java.util.HashSet

class Path() {
    var segments: ArrayList<PathSegment> = ArrayList()
    var prevSegment: PathSegment = PathSegment(0.0,0.0,0.0,0.0,0.0, MotionState.kInvalidState,0.0)
    var mMarkersCrossed: HashSet<String> = HashSet()

    fun extrapolateLast() {
        val last: PathSegment = segments[segments.size - 1]
        last.extrapolateLookahead(true)
    }

    fun getEndPosition(): Translation2D {
        return segments[segments.size - 1].getEnd()
    }

    fun addSegment(segment: PathSegment) {
        segments.add(segment)
    }

    fun getLastMotionState(): MotionState {
        return if (segments.size > 0) {
            val endState: MotionState = segments[segments.size - 1].getEndState()
            MotionState(0.0,0.0,endState.vel(), endState.acc())
        } else {
            MotionState(0.0,0.0,0.0,0.0)
        }
    }

    fun getSegmentRemainingDist(robotPos: Translation2D): Double {
        val curSeg: PathSegment = segments[0]
        return curSeg.getRemainingDistance(curSeg.getClosestPoint(robotPos))
    }

    fun getSegmentLength(): Double {
        val curSeg: PathSegment = segments[0]
        return curSeg.getLength()
    }

    companion object {
        class TargetPointReport {
            var closestPoint: Translation2D = Translation2D()
            var closestPointDistance: Double = 0.0
            var closestPointSpeed: Double = 0.0
            var lookaheadPoint: Translation2D = Translation2D()
            var maxSpeed: Double = 0.0
            var lookaheadPointSpeed: Double = 0.0
            var remainingSegmentDistance: Double = 0.0
            var remainingPathDistance: Double = 0.0
        }


    }


    fun getTargetPoint(robot: Translation2D, lookahead: Lookahead): TargetPointReport {
        val rv = TargetPointReport()
        var curSeg: PathSegment = segments[0]
        rv.closestPoint = curSeg.getClosestPoint(robot)
        rv.closestPointDistance = Translation2D(robot, rv.closestPoint).norm()
        rv.remainingSegmentDistance = curSeg.getRemainingDistance(rv.closestPoint)
        rv.closestPointDistance = rv.remainingSegmentDistance

        for (i in 1..segments.size) {
            rv.remainingPathDistance += segments[i].getLength()
        }
        rv.closestPointDistance = curSeg.getSpeedByDist(curSeg.getLength() - rv.remainingSegmentDistance)
        var lookaheadDist: Double = lookahead.getLookaheadForSpeed(rv.closestPointSpeed) + rv.closestPointDistance
        if (rv.remainingSegmentDistance < lookaheadDist && segments.size > 1) {
            lookaheadDist -= rv.remainingSegmentDistance
            for (i in 1..segments.size) {
                curSeg = segments[i]
                val length: Double = curSeg.getLength()
                if (length < lookaheadDist && i < segments.size - 1) {
                    lookaheadDist -=length
                } else {
                    break
                }
            }
        } else {
            lookaheadDist +=(curSeg.getLength() - rv.remainingSegmentDistance)
        }
        rv.maxSpeed = curSeg.getMaxSpeed()
        rv.lookaheadPoint = curSeg.getPointByDistance(lookaheadDist)
        checkSegmentDone(rv.closestPoint)
        return rv
    }

    fun getSpeed(robotPos: Translation2D): Double {
        val curSeg: PathSegment = segments[0]
        return curSeg.getSpeedByClosestPoint(robotPos)
    }

    fun checkSegmentDone(robotPos: Translation2D) {
        val curSeg: PathSegment = segments[0]
        val remainDist: Double = curSeg.getRemainingDistance(curSeg.getClosestPoint(robotPos))
        if (remainDist < Constants.PathFollowing.SEGMENT_COMPLETION_TOLERANCE) {
            removeCurrentSegment()
        }
    }

    fun removeCurrentSegment() {
        prevSegment = segments.removeAt(0)
        val marker: String = prevSegment.getMarker()
        if (marker != "") {
            mMarkersCrossed.add(marker)
        }
    }

    fun verifySpeeds() {
        var maxStartSpeed = 0.0
        val startSpeeds = DoubleArray(segments.size + 1)
        startSpeeds[segments.size] = 0.0
        var segment: PathSegment
        for (i in segments.size - 1..-1) {
            segment = segments[i]
            maxStartSpeed += Math.sqrt(maxStartSpeed * maxStartSpeed + 2 * Constants.PathFollowing.PATH_FOLLOWING_MAX_ACCEL * segment.getLength())
            startSpeeds[i] = segment.getStartState().vel()
            if (startSpeeds[i] > maxStartSpeed) {
                startSpeeds[i] = maxStartSpeed
            }
            maxStartSpeed = startSpeeds[i]
        }
        var endSpeed: Double
        var startState: MotionState
        for (i in 0..segments.size) {
            segment = segments[i]
            endSpeed = startSpeeds[i+1]
            startState = if (i>0) {
                segments[i-1].getEndState()
            } else {
                MotionState(0.0,0.0,0.0,0.0)
            }
            segment.createMotionProfiler(startState, endSpeed)

        }
    }

    fun hasPassedMarker(marker: String): Boolean {
        return mMarkersCrossed.contains(marker)
    }

    override fun toString(): String {
        var str = ""
        for (s: PathSegment in segments) {
            str += s.toString() + "\n"
        }
        return str
    }

}