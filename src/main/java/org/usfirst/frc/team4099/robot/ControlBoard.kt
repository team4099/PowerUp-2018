package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.DualShock4Gamepad
import org.usfirst.frc.team4099.lib.joystick.Gamepad
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils

class ControlBoard private constructor() {
    private val driver: Gamepad = DualShock4Gamepad(Constants.Joysticks.DRIVER_PORT)
    private val operator: Gamepad = DualShock4Gamepad(Constants.Joysticks.SHOTGUN_PORT)

    val throttle: Double
        get() = driver.rightTriggerAxis - driver.leftTriggerAxis

    val turn: Double
        get() = driver.leftXAxis

    val switchToHighGear: Boolean
        get() = driver.rightShoulderButton

    val switchToLowGear: Boolean
        get() = driver.leftShoulderButton
    /**
     * Should the bot arcadeDrive in quick turn mode?
     * @return  true/false, depending on if the joystick is depressed
     */
    val quickTurn: Boolean
        get() = Math.abs(JoystickUtils.deadbandNoShape(throttle, 0.02)) < 0.01

    val reverseIntake: Boolean
        get() = driver.aButton

    val test: Boolean
        get() = driver.bButton

    companion object {
        val instance = ControlBoard()
    }

}
