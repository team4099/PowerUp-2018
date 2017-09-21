package org.usfirst.frc.team4099.lib.joystick

import edu.wpi.first.wpilibj.Joystick

/**
 * Controller Settings for Correct Mappings
 * ----------------------------------------
 * X Emulation Mode (switch on back = X)
 * Flight Mode (Mode Light = Off)
 */

class LogitechF310Gamepad(port: Int) : Joystick(port) {

    val leftXAxis: Double
        get() = this.getRawAxis(0)

    val leftYAxis: Double
        get() = this.getRawAxis(1)

    val leftTriggerAxis: Double
        get() = this.getRawAxis(2)

    val rightTriggerAxis: Double
        get() = this.getRawAxis(3)

    val rightXAxis: Double
        get() = this.getRawAxis(4)

    val rightYAxis: Double
        get() = this.getRawAxis(5)

    val aButton: Boolean
        get() = this.getRawButton(1)

    val bButton: Boolean
        get() = this.getRawButton(2)

    val xButton: Boolean
        get() = this.getRawButton(3)

    val yButton: Boolean
        get() = this.getRawButton(4)

    val leftJoystickButton: Boolean
        get() = this.getRawButton(9)

    val rightJoystickButton: Boolean
        get() = this.getRawButton(10)

    val leftShoulderButton: Boolean
        get() = this.getRawButton(5)

    val rightShoulderButton: Boolean
        get() = this.getRawButton(6)

    val dPadUp: Boolean
        get() = this.pov == 0 || this.pov == 360

    val dPadDown: Boolean
        get() = this.pov == 180

    val dPadLeft: Boolean
        get() = this.pov == 90

    val dPadRight: Boolean
        get() = this.pov == 270
}
