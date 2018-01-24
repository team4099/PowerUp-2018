package org.usfirst.frc.team4099.lib.util

class InterpolatingLong(valu: Long) : Interpolable<InterpolatingLong>, InverseInterpolable<InterpolatingLong>, Comparable<InterpolatingLong> {
    var value: Long = valu

    override fun interpolate(other: InterpolatingLong, x: Double): InterpolatingLong {
        val dydx: Long = other.value - value
        val searchY: Double = dydx * x + value
        return InterpolatingLong(searchY.toLong())
    }

    override fun inverseInterpolate(upper: InterpolatingLong, query: InterpolatingLong): Double {
        val upToLow: Long = upper.value - value
        if (upToLow <= 0) {
            return 0.0
        }
        val queryToLow: Long = query.value - value
        if (queryToLow <= 0) {
            return 0.0
        }
        return queryToLow / upToLow.toDouble()
    }

    override fun compareTo(other: InterpolatingLong): Int {
        return when {
            other.value < value -> 1
            other.value > value -> -1
            else -> 0
        }
    }
}