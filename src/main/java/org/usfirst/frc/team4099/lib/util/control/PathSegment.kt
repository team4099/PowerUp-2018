package org.usfirst.frc.team4099.lib.util.control

import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D
import org.usfirst.frc.team4099.lib.util.motion.MotionProfile
import org.usfirst.frc.team4099.lib.util.motion.MotionProfileConstraints
import org.usfirst.frc.team4099.lib.util.motion.MotionProfileGenerator
import org.usfirst.frc.team4099.lib.util.motion.MotionProfileGoal
import org.usfirst.frc.team4099.lib.util.motion.MotionState
import org.usfirst.frc.team4099.robot.Constants

import java.util.Optional

class PathSegment {
    private var start: Translation2D = Translation2D()
    private var end: Translation2D = Translation2D()
    private var center: Translation2D = Translation2D()
    private var dStart: Translation2D = Translation2D()
    private var dEnd: Translation2D = Translation2D()
    private var maxSpeed: Double = 0.0
    private var isLine: Boolean = false
    private var speedCont: MotionProfile = MotionProfile()
    private var extrapolateLookahead: Boolean = false
    private var marker: String = ""

    constructor(x1: Double, y1: Double, x2: Double, y2: Double, maxS: Double, sState: MotionState, endS: Double) {
        start = Translation2D(x1, y1)
        end = Translation2D(x2, y2)

        dStart = Translation2D(start, end)
        maxSpeed = maxS
        extrapolateLookahead = false
        isLine = true
        createMotionProfiler(sState, endS)
    }

    constructor(x1: Double, y1: Double, x2: Double, y2: Double, maxS: Double, sState: MotionState, endS: Double, m: String) {
        start = Translation2D(x1, y1)
        end = Translation2D(x2, y2)

        dStart = Translation2D(start, end)

        maxSpeed = maxS
        extrapolateLookahead = false
        isLine = true
        marker = m
        createMotionProfiler(sState, endS)
    }

    constructor(x1: Double, y1: Double, x2: Double, y2: Double, cx: Double, cy: Double, maxS: Double, sState: MotionState, endS: Double) {
        start = Translation2D(x1, y1)
        end = Translation2D(x2, y2)
        center = Translation2D(cx, cy)
        dStart = Translation2D(center, start)
        dEnd = Translation2D(center, end)

        maxSpeed = maxS
        extrapolateLookahead = false
        isLine = false
        createMotionProfiler(sState, endS)
    }

    constructor(x1: Double, y1: Double, x2: Double, y2: Double, cx: Double, cy: Double, maxS: Double, sState: MotionState, endS: Double, mark: String) {
        start = Translation2D(x1, y1)
        end = Translation2D(x2, y2)
        center = Translation2D(cx, cy)
        dStart = Translation2D(center, start)
        dEnd = Translation2D(center, end)

        maxSpeed = maxS
        extrapolateLookahead = false
        isLine = false
        marker = mark
        createMotionProfiler(sState, endS)
    }

    fun getMaxSpeed(): Double {
        return maxSpeed
    }

    fun createMotionProfiler(start_s: MotionState, end_sp: Double) {
        var mC: MotionProfileConstraints = MotionProfileConstraints(maxSpeed, Constants.PathFollowing.PATH_FOLLOWING_MAX_ACCEL)
        var goal_state: MotionProfileGoal = MotionProfileGoal(getLength(), end_sp)
        speedCont = MotionProfileGenerator.generateProfile(mC, goal_state, start_s)
    }

    fun getStart(): Translation2D {
        return start
    }

    fun getEnd(): Translation2D {
        return end
    }

    fun getLength(): Double {
        if (isLine) {
            return dStart.norm()
        } else {
            return dStart.norm() * Translation2D.getAngle(dStart,dEnd).radians
        }
    }

    fun extrapolateLookahead(value: Boolean) {
        extrapolateLookahead = value
    }

    fun getClosestPoint(position: Translation2D): Translation2D {
        if (isLine) {
            var delta: Translation2D = Translation2D(start, end)
            var u: Double = ((position.x() - start.x()) * delta.x() + (position.y() - start.y()) * delta.y()) / (delta.x() * delta.x() + delta.y() * delta.y())
            if (u >= 0 && u <= 1)
                return Translation2D(start.x() + u * delta.x(), start.y() + u * delta.y());
            if (u < 0) {
                return start
            } else {
                return end
            }
        } else {
            var deltaPosition: Translation2D = Translation2D(center, position)
            deltaPosition = deltaPosition.scale(dStart.norm() / deltaPosition.norm());
            if (Translation2D.cross(deltaPosition, dStart) * Translation2D.cross(deltaPosition, dEnd) < 0) {
                return center.translateBy(deltaPosition)
            } else {
                var startDist: Translation2D = Translation2D(position, start)
                var endDist: Translation2D = Translation2D(position, end)
                if (endDist.norm() < startDist.norm()) {
                    return end
                } else {
                    return start
                }
            }
        }
    }

    fun getPointByDistance(dista: Double): Translation2D {
        var length: Double = getLength()
        var dist: Double = dista
        if (!extrapolateLookahead && dist > length) {
            dist = length
        }
        if (isLine) {
            return start.translateBy(dStart.scale(dist / length))
        } else {
            var dAngle: Double = Translation2D.getAngle(dStart, dEnd).radians
            if (Translation2D.cross(dStart, dEnd) <0) {
                dAngle *= -1
            }
            dAngle *= dist / length
            var t: Translation2D = dStart.rotateBy(Rotation2D.fromRadians(dAngle))
            return center.translateBy(t)
        }
    }

    fun getRemainingDistance(pos: Translation2D) : Double {
        if (isLine) {
            return Translation2D(end, pos).norm()
        } else {
            var dPos: Translation2D = Translation2D(center, pos)
            var ang: Double = Translation2D.getAngle(dEnd, dPos).radians
            var totAng: Double = Translation2D.getAngle(dStart, dEnd).radians
            return ang / totAng * getLength()
        }
    }

    fun getDistanceTraveled(robotPos: Translation2D): Double {
        var pathPos: Translation2D = getClosestPoint(robotPos)
        var remainDist: Double = getRemainingDistance(pathPos)
        return getLength() - remainDist
    }

    fun getSpeedByDist(dista: Double): Double {
        var dist: Double = dista
        if (dist < speedCont.startPos()) {
            dist = speedCont.startPos()
        } else if (dist > speedCont.endPos()) {
            dist = speedCont.endPos()
        }
        var state: Optional<MotionState> = speedCont.firstStateByPos(dist)
        if (state.isPresent) {
            return state.get().vel()
        } else {
            System.out.println("Velocity dne")
            return 0.0
        }
    }

    fun getSpeedByClosestPoint(robotPos: Translation2D): Double {
        return getSpeedByDist(getDistanceTraveled(robotPos))
    }

    fun getEndState(): MotionState {
        return speedCont.endState()
    }

    fun getStartState(): MotionState {
        return speedCont.startState()
    }

    fun getMarker(): String {
        return marker
    }

    override fun toString(): String {
        if (isLine) {
            return "(" + "start: " + start + ", end: " + end + ", speed: " + maxSpeed + ")"
        } else {
            return "(" + "start: " + start + ", end: " + end + ", center: " + center + ", speed: " + maxSpeed + ")"; // + ", profile: " + speedController + ")"
        }
    }
}