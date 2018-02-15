package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.DualShock4Gamepad
import org.usfirst.frc.team4099.lib.joystick.Gamepad
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils
import org.usfirst.frc.team4099.lib.joystick.XboxOneGamepad

class ControlBoard private constructor() {
    private val driver: Gamepad = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
    private val operator: Gamepad = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)

    val throttle: Double
        get() = driver.rightTriggerAxis - driver.leftTriggerAxis

    val turn: Double
        get() = driver.leftXAxis

    val switchToHighGear: Boolean
        get() = driver.rightShoulderButton

    val switchToLowGear: Boolean
        get() = driver.leftShoulderButton

    val openIntake: Boolean
        get() = operator.dPadLeft

    val closeIntake: Boolean
        get() = operator.dPadRight

    /**
     * Should the bot arcadeDrive in quick turn mode?
     * @return  true/false, depending on if the joystick is depressed
     */
    val quickTurn: Boolean
        get() = Math.abs(JoystickUtils.deadbandNoShape(throttle, 0.02)) < 0.01

    val reverseIntake: Boolean
        get() = operator.aButton

    val test: Boolean
        get() = driver.bButton

    val elevatorTop: Boolean
        get() = operator.dPadUp

    val elevatorBottom: Boolean
        get() = operator.dPadDown

    val elevatorPower: Double
        get() = operator.rightTriggerAxis - operator.leftTriggerAxis

    companion object {
        val instance = ControlBoard()
    }

}
