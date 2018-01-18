package org.usfirst.frc.team4099.robot.subsystems

class WristConversion private constructor(): EncoderConversion {
    val pulsesToRadians = 1.0

    override fun radiansToPulses(radians: Double): Int {
        return (radians / pulsesToRadians).toInt()
    }

    override fun pulsesToRadians(pulses: Int): Double {
        return pulses * pulsesToRadians
    }

    companion object {
        val instance = WristConversion()
    }
}