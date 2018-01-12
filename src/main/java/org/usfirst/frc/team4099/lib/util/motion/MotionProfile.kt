package org.usfirst.frc.team4099.lib.util.motion

import org.usfirst.frc.team4099.lib.util.Utils.epsilonEquals
import org.usfirst.frc.team4099.lib.util.motion.MotionUtil.kEpsilon

import java.util.ArrayList
import kotlin.collections.Iterator
import kotlin.collections.List
import java.util.Optional

class MotionProfile {
    protected var mSegments_: List<MotionSegment>

    constructor() {
        mSegments_ = ArrayList<MotionSegment>()
    }

    constructor(segments: List<MotionSegment>) {
        mSegments_ = segments
    }

    fun isValid(): Boolean {
        var prev_seg: MotionSegment = null
        for (s: MotionSegment in mSegments_) {
            if (!s.isValid()) {
                return false
            }
            if (prev_seg != null && !s.start().coincident(prev_seg.end())) {
                System.err.println("Segments not continuous! End is " + prev_seg.end() + ", Start is " + s.start())
                return false
            }
            prev_seg = s
        }
        return true
    }

    fun isEmpty(): Boolean {
        return mSegments_.isEmpty()
    }

    fun stateByTime(t: Double): Optional<MotionState> {
        if (t < startTime() && t + kEpsilon >= startTime()) {
            return Optional.of(startState())
        }

        if (t > endTime() && t - kEpsiln >= endTime()) {
            return Optional.of(endState())
        }

        for (s: MotionSegment in mSegments_) {
            if (s.containsTime(t)) {
                return Optional.of(s.start().extrapolate(t))
            }
        }

        return Optional.empty()
    }

    fun stateByTimeClamped(t: Double): MotionState {
        if (t < startTime()) {
            return startState()
        } else if (t > endTime()) {
            return endState()
        }
        for (s: MotionSegment in mSegments_) {
            if (s.containsTime(t)) {
                return s.start() extrapolate(t)
            }
        }

        return MotionState.kInvalidState
    }

    fun firstStateByPos(pos: Double): Optional<MotionState> {
        for (s: MotionSegment in mSegments_) {
            if (s.containsPos(pos)) {
                if (epsilonEquals(s.end().pos(), pos, kEpsilon)) {
                    return Optional.of(s.end())
                }
                val t: Double = Math.min(s.start().nextTimeAtPos(pos), s.end().t())
                if (t.isNaN()) {
                    System.err.println("Error: should reach pos but dont")
                    return Optional.empty()
                }
                return Optional.of(s.start().extrapolate(t))
            }
        }
        return Optional.empty()
    }

    fun trimBeforeTime(t: Double) {
        var iterator: Iterator<MotionSegment> = mSegments_.iterator()
        while (iterator.hasNext()){
            var s: MotionSegment = iterator.next()
            if (s.end().t() <=t) {
                iterator.remove()
                continue
            }
        }
    }

    fun clear() {
        mSegments_.clear()
    }

    fun reset(init_state: MotionState) {
        clear()
        mSegments_.add(MotionSegment(init_state,init_state))
    }

    fun consolidate() {
        var iterator: Iterator<MotionSegment> = mSegments_.iterator()
        while (iterator.hasNext() && mSegments_.size() > 1) {
            var s: MotionSegment = iterator.next()
            if(s.start().coincident(s.end())){
                iterator.remove()
            }
        }
    }

    fun appendControl(acc: Double, dt: Double) {
        if (isEmpty()) {
            System.err.println("Error: appending to empty profile")
            return
        }
        var last_end_state: MotionState = mSegments_.get(mSegments_.size()-1).end()
        var new_start_state: MotionState = MotionState(last_end_state.t(), last_end_state.pos(), last_end_state.vel(), acc)
        appendSegment(MotionSegment(new_start_state, new_start_state.extrapolate(new_start_state.t()+dt)))
    }

    fun appendSegment(segment: MotionSegment) {
        mSegments_.add(segment)
    }

    fun appendProfile(profile: MotionProfile) {
        for (s: MotionSegment in profile.segments())
            appendSegment(s)
    }

    fun size(): Int {
        return mSegments_.size
    }

    fun segments(): List<MotionSegment> {
        return mSegments_
    }

    fun startState(): MotionState {
        if (isEmpty()) {
            return MotionState.kInvalidState
        }
        return mSegments_.get(0).start()
    }

    fun startTime(): Double {
        return startState().t()
    }

    fun startPos(): Double {
        return startState().pos()
    }

    fun endState(): MotionState {
        if (isEmpty()) {
            return MotionState.kInvalidState
        }
        return mSegments_.get(mSegments_.size()-1).end()
    }

    fun endTime(): Double {
        return endState().t()
    }

    fun endPos(): Double {
        return endState().pos()
    }

    fun duration(): Double {
        return endTime() - startTime()
    }

    fun length(): Double {
        var length: Double = 0.0
        for (s: MotionSegment in segments()){
            length+=Math.abs(s.end().pos() - s.start().pos())
        }
        return length
    }

    override fun toString(): String {
        var builder: StringBuilder = StringBuilder("Profile:")
        for (s: MotionSegment in segments()) {
            builder.append("\n\t")
            builder.append(s)
        }
        return builder.toString()
    }
}