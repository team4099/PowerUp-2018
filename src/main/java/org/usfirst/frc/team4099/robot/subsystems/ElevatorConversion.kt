package org.usfirst.frc.team4099.robot.subsystems

object ElevatorConversion : EncoderConversion {
    val pulsesToRadians = 1.0

    override fun radiansToPulses(radians: Double): Int {
        return (radians / pulsesToRadians).toInt()
    }

    override fun pulsesToRadians(pulses: Int): Double {
        return pulses * pulsesToRadians
    }
}