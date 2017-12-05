package org.usfirst.frc.team4099.lib.joystick

import edu.wpi.first.wpilibj.Joystick

abstract class SteeringWheel(port: Int): Joystick(port) {

    abstract val accelPedal: Double

    abstract val brakePedal: Double

    abstract val steer: Double
}