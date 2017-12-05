package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.DualShock4Gamepad
import org.usfirst.frc.team4099.lib.joystick.Gamepad
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils
import org.usfirst.frc.team4099.lib.joystick.XboxOneGamepad

class GamepadControlBoard private constructor(): ControlBoard {
    private val driver: Gamepad = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
    private val operator: Gamepad = DualShock4Gamepad(Constants.Joysticks.SHOTGUN_PORT)

    override val throttle: Double
        get() = -driver.rightTriggerAxis + driver.leftTriggerAxis

    override val turn: Double
        get() = driver.leftXAxis

    override val toggleSlowMode: Boolean
        get() = driver.aButton

    /**
     * Should the bot arcadeDrive in quick turn mode?
     * @return  true/false, depending on if the joystick is depressed
     */
    override val quickTurn: Boolean
        get() = Math.abs(JoystickUtils.deadbandNoShape(throttle, 0.02)) < 0.01

    override val intakeUp: Boolean
        get() = operator.dPadUp

    override val intakeDown: Boolean
        get() = operator.dPadDown

    override val toggleIntake: Boolean
        get() = operator.aButton

    override val climber: Boolean
        get() = operator.yButton

    override val toggleIntakeClosed: Boolean
        get() = operator.dPadRight || operator.dPadLeft

    companion object {
        val instance = GamepadControlBoard()
    }

}
