package org.usfirst.frc.team4099.robot;

import org.usfirst.frc.team4099.lib.joystick.DualShock4Gamepad;
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils;
import org.usfirst.frc.team4099.lib.joystick.XboxOneGamepad;

public class ControlBoard {
    private static ControlBoard sInstance = new ControlBoard();
    public static ControlBoard getInstance() {
        return sInstance;
    }
    private final XboxOneGamepad driver;
    private final DualShock4Gamepad operator;

    private ControlBoard() {
        driver = new XboxOneGamepad(Constants.Joysticks.DRIVER_PORT);
        operator = new DualShock4Gamepad(Constants.Joysticks.SHOTGUN_PORT);
    }

    public double getThrottle() {
        return -driver.getRightTriggerAxis() + driver.getLeftTriggerAxis();
    }

    public double getTurn() {
        return driver.getLeftXAxis();
    }

    /**
     * Should the bot drive in quick turn mode?
     * @return  true/false, depending on if the joystick is depressed
     */
    public boolean getQuickTurn() {
        return Math.abs(JoystickUtils.deadbandNoShape(getThrottle(), 0.02)) < 0.01;
        //return driver.getXButton();
    }

    public boolean getToggleIntakeUp() {
        return driver.getBButton();
    }

    public boolean getToggleIntake() {
        return operator.getAButton();
    }

    public boolean getClimber() {
        return operator.getYButton();
    }
}
