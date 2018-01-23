package org.usfirst.frc.team4099.lib.util

class InterpolatingDouble(valu: Double) : Interpolable<InterpolatingDouble>, InverseInterpolable<InterpolatingDouble>, Comparable<InterpolatingDouble> {
    var value: Double = valu

    override fun interpolate(other: InterpolatingDouble, x: Double): InterpolatingDouble {
        val dydx: Double = other.value - value
        val searchY: Double = dydx * x + value
        return InterpolatingDouble(searchY)
    }

    override fun inverseInterpolate(upper: InterpolatingDouble, query: InterpolatingDouble): Double {
        val upToLow: Double = upper.value - value
        if (upToLow <= 0) {
            return 0.0
        }
        val queryToLow: Double = query.value - value
        if (queryToLow <= 0) {
            return 0.0
        }
        return queryToLow / upToLow
    }

    override fun compareTo(other: InterpolatingDouble): Int {
        return when {
            other.value < value -> 1
            other.value > value -> -1
            else -> 0
        }
    }
}