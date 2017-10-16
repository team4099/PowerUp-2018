package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.DualShock4Gamepad
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils
import org.usfirst.frc.team4099.lib.joystick.LogitechF310Gamepad

class ControlBoard private constructor() {
    private val driver: LogitechF310Gamepad = LogitechF310Gamepad(Constants.Joysticks.DRIVER_PORT)
    private val operator: DualShock4Gamepad = DualShock4Gamepad(Constants.Joysticks.SHOTGUN_PORT)

    val throttle: Double
        get() = -driver.rightTriggerAxis + driver.leftTriggerAxis

    val turn: Double
        get() = driver.leftXAxis

    val toggleSlowMode: Boolean
        get() = driver.aButton

    /**
     * Should the bot arcadeDrive in quick turn mode?
     * @return  true/false, depending on if the joystick is depressed
     */
    //return driver.getXButton();
    val quickTurn: Boolean
        get() = Math.abs(JoystickUtils.deadbandNoShape(throttle, 0.02)) < 0.01

    val intakeUp: Boolean
        get() = operator.dPadUp

    val intakeDown: Boolean
        get() = operator.dPadDown

    val toggleIntake: Boolean
        get() = operator.aButton

    val climber: Boolean
        get() = operator.yButton

    val toggleIntakeClosed: Boolean
        get() = operator.dPadRight || operator.dPadLeft

    companion object {
        val instance = ControlBoard()
    }

}
