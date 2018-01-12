package org.usfirst.frc.team4099.lib.util.motion

import org.usfirst.frc.team4099.lib.util.Utils.epsilonEquals
import org.usfirst.frc.team4099.lib.util.motion.motionUtil.kEpsilon

class MotionSegment {
    protected var mStart_: MotionState = MotionState(0.0,0.0,0.0,0.0)
    protected var mEnd_: MotionState = MotionState(0.0,0.0,0.0,0.0)

    constructor(start: MotionState, end: MotionState) {
        mStart_ = start
        mEnd_ = end
    }

    fun isValid(): Boolean {
        if (!epsilonEquals(start().acc(), end().acc(), kEpsilon)) {
            System.err.println("Segment acceleration not constant! Start acc: " + start().acc() + ", End acc: " + end().acc())
            return false
        }
        if (Math.signum(start().vel()) * Math.signum(end().vel()) < 0.0 && !epsilonEquals(start().vel(), 0.0, kEpsilon) && !epsilonEquals(end().vel(), 0.0, kEpsilon)) {
            System.err.println("Segment velocity reverses! Start vel: " + start().vel() + ", End vel: " + end().vel());
            return false
        }

        if (!start().extrapolate(end().t()).equals(end())) {
            if (start().t() == end().t() && (start().acc() == Double.POSITIVE_INFINITY || start().acc() == Double.NEGATIVE_INFINITY)) {
                return true
            }
            System.err.println("Segment not consistent! Start: " + start() + ", End: " + end());
            return false
        }
        return true
    }

    fun containsTime(t: Double): Boolean {
        return t>=start().t() && t<=end().t()
    }

    fun containsPos(pos: Double): Boolean {
        return pos>=start().pos() && pos<=end().pos() || pos <= start().pos() && pos >= end().pos()
    }

    fun start(): MotionState {
        return mStart_
    }

    fun setStart(start: MotionState) {
        mStart_ = start
    }

    fun end(): MotionState {
        return mEnd_
    }

    fun setEnd(end: MotionState) {
        mEnd_ = end
    }

    override fun toString(): String {
        return "Start: "+ start() + ", End: " + end()
    }
}