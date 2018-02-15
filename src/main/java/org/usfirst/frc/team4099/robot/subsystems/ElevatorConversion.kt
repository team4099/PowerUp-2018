package org.usfirst.frc.team4099.robot.subsystems

object ElevatorConversion : EncoderConversion {
    val pulsesToInches = 1.0/907.0

    override fun inchesToPulses(inches: Double): Int {
        return (inches / pulsesToInches).toInt()
    }

    override fun pulsesToInches(pulses: Int): Double {
        return pulses * pulsesToInches
    }

    override fun inchesPerSecondtoNativeSpeed(ips: Double): Int {
        return inchesToPulses(ips) / 10
    }

    override fun nativeSpeedToInchesPerSecond(nativeSpeed: Int): Double {
        return pulsesToInches(nativeSpeed) * 10
    }
}