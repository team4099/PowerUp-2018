package org.usfirst.frc.team4099.lib.joystick


class ThrustmasterWheel(port: Int): SteeringWheel(port) {

    override val accelPedal: Double
        get() = this.getRawAxis(1)

    override val brakePedal: Double
        get() = this.getRawAxis(2)

    override val steer: Double
        get() = this.getRawAxis(0)
}