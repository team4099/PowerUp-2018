package org.usfirst.frc.team4099.lib.util.control

class Lookahead {
    val min_dist: Double
    val max_dist: Double
    val min_speed: Double
    val max_speed: Double

    protected val d_dist: Double
    protected val d_speed: Double

    constructor(mind: Double, maxd: Double, mins: Double, maxs: Double) {
        min_dist = mind
        max_dist = maxd
        min_speed = mins
        max_speed = maxs

        d_dist = max_dist - min_dist
        d_speed = max_speed - min_speed
    }

    fun getLookaheadForSpeed(s: Double): Double {
        var lookahead: Double = d_dist * (s - min_speed) / d_speed + min_dist
        if (lookahead.isNaN()) {
            lookahead = min_dist
        } else {
            lookahead = Math.max(min_dist, Math.min(max_dist, lookahead))
        }
        return lookahead
    }
}