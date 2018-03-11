package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.*

class ControlBoard private constructor() {
    private val driver: Gamepad = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
    private val operator: Gamepad = LogitechF310Gamepad(Constants.Joysticks.SHOTGUN_PORT)

    val throttle: Double
        get() = driver.rightTriggerAxis - driver.leftTriggerAxis

    val turn: Double
        get() = -driver.leftXAxis

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

    val wristTop: Boolean
        get() = driver.dPadUp

    val wristBottom: Boolean
        get() = driver.dPadDown

    val elevatorPower: Double
        get() = operator.rightTriggerAxis - operator.leftTriggerAxis

    val wristPower: Double
        get() = driver.rightTriggerAxis - driver.leftTriggerAxis

    companion object {
        val instance = ControlBoard()
    }

}
