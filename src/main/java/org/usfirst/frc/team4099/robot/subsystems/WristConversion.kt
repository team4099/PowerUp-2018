package org.usfirst.frc.team4099.robot.subsystems


object WristConversion: EncoderConversion {
    override fun inchesPerSecondtoNativeSpeed(ips: Double): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun nativeSpeedToInchesPerSecond(nativeSpeed: Int): Double {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    val pulsesToRadians = 1.0

    override fun inchesToPulses(radians: Double): Int {
        return (radians / pulsesToRadians).toInt()
    }

    override fun pulsesToInches(pulses: Int): Double {
        return pulses * pulsesToRadians
    }
}