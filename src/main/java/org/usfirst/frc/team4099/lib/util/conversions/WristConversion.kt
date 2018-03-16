package org.usfirst.frc.team4099.lib.util.conversions


object WristConversion {
    val pulsesToRadians = 0.055

    fun radiansToPulses(radians: Double): Double {
        return (radians / pulsesToRadians)
    }

    fun pulsesToRadians(pulses: Double): Double {
        return pulses * pulsesToRadians
    }
}