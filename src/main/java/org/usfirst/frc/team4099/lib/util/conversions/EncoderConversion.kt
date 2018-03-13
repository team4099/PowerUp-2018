package org.usfirst.frc.team4099.robot.subsystems

interface EncoderConversion {
    fun inchesToPulses(radians: Double): Int
    fun pulsesToInches(pulses: Int): Double
    fun inchesPerSecondtoNativeSpeed(ips: Double): Int
    fun nativeSpeedToInchesPerSecond(nativeSpeed: Int): Double
}