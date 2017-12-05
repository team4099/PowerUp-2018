package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.JoystickUtils
import org.usfirst.frc.team4099.lib.joystick.SteeringWheel
import org.usfirst.frc.team4099.lib.joystick.ThrustmasterWheel
import org.usfirst.frc.team4099.lib.joystick.XboxOneGamepad

class SteeringWheelControlBoard private constructor(): ControlBoard {
    private val steeringWheel: SteeringWheel = ThrustmasterWheel(Constants.Joysticks.DRIVER_PORT)
    private val operator: XboxOneGamepad = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)

    override val throttle: Double
        get() = steeringWheel.accelPedal - steeringWheel.brakePedal

    override val turn: Double
        get() = steeringWheel.steer

    override val toggleSlowMode: Boolean
        get() = steeringWheel.getRawButton(3)

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
        val instance = SteeringWheelControlBoard()
    }

}