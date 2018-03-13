package org.usfirst.frc.team4099.robot.subsystems


object WristConversion {
    val pulsesToRadians = 0.055

    fun radiansToPulses(radians: Double): Int {
        return (radians / pulsesToRadians).toInt()
    }

    fun pulsesToRadians(pulses: Int): Double {
        return pulses * pulsesToRadians
    }
}