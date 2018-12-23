package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.Gamepad
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils
import org.usfirst.frc.team4099.lib.joystick.XboxOneGamepad

class ControlBoard private constructor() {
    private val driver: Gamepad = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
    private val operator: Gamepad = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)

    val throttle: Double
        get() = -driver.rightTriggerAxis + driver.leftTriggerAxis

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

    val reverseIntakeSlow: Boolean
        get() = operator.bButton

    val reverseIntakeFast: Boolean
        get() = operator.yButton

    val runIntake: Boolean
        get() = operator.aButton

    val test: Boolean
        get() = operator.yButton

//    val elevatorTop: Boolean
//        get() = operator.dPadUp
//
//    val elevatorBottom: Boolean
//    val wristTop: Boolean
//        get() = driver.dPadUp
//
//    val wristBottom: Boolean

    val deployForks: Boolean
        get() = operator.rightShoulderButton && operator.leftShoulderButton

    val unClimber: Boolean
        get() = operator.xButton && operator.rightJoystickButton && operator.leftShoulderButton
                && operator.leftJoystickButton

    val runClimber: Boolean
        get() = operator.xButton && !unClimber

//        get() = operator.dPadDown

//        get() = driver.dPadDown

    val elevatorPower: Double
        get() = (operator.rightTriggerAxis - operator.leftTriggerAxis) * 1

    val wristPower: Double
        get() = operator.leftYAxis

    companion object {
        val instance = ControlBoard()
    }

}
