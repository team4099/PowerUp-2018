package org.usfirst.frc.team4099.lib.joystick

import edu.wpi.first.wpilibj.Joystick

abstract class Gamepad(port: Int): Joystick(port) {
    abstract val leftXAxis: Double

    abstract val leftYAxis: Double

    abstract val leftTriggerAxis: Double

    abstract val rightTriggerAxis: Double

    abstract val rightXAxis: Double

    abstract val rightYAxis: Double

    abstract val aButton: Boolean

    abstract val bButton: Boolean

    abstract val xButton: Boolean

    abstract val yButton: Boolean

    abstract val leftJoystickButton: Boolean

    abstract val rightJoystickButton: Boolean

    abstract val leftShoulderButton: Boolean

    abstract val rightShoulderButton: Boolean

    abstract val dPadUp: Boolean

    abstract val dPadDown: Boolean

    abstract val dPadLeft: Boolean

    abstract val dPadRight: Boolean
}