package org.usfirst.frc.team4099.robot.subsystems

interface EncoderConversion {
    fun radiansToPulses(radians: Double): Int
    fun pulsesToRadians(pulses: Int): Double
}