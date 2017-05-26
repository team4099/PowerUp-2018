package org.usfirst.frc.team4099.robot;

import org.usfirst.frc.team4099.lib.joystick.JoystickUtils;
import org.usfirst.frc.team4099.lib.joystick.LogitechF310Gamepad;

public class ControlBoard {
    private static ControlBoard sInstance = new ControlBoard();
    public static ControlBoard getInstance() {
        return sInstance;
    }
    private final LogitechF310Gamepad driver;
//    private final DualShock4Gamepad operator;

    private ControlBoard() {
        driver = new LogitechF310Gamepad(Constants.Joysticks.DRIVER_PORT);
//        driver = new DualShock4Gamepad(Constants.Joysticks.DRIVER_PORT);
//        operator = new DualShock4Gamepad(Constants.Joysticks.SHOTGUN_PORT);
    }

    public double getThrottle() {
        return -driver.getRightTriggerAxis() + driver.getLeftTriggerAxis();
    }

    public double getTurn() {
        return driver.getLeftXAxis();
    }

    public boolean getToggleSlowMode() {
        return driver.getAButton();
    }

    /**
     * Should the bot arcadeDrive in quick turn mode?
     * @return  true/false, depending on if the joystick is depressed
     */
    public boolean getQuickTurn() {
        return Math.abs(JoystickUtils.deadbandNoShape(getThrottle(), 0.02)) < 0.01;
        //return driver.getXButton();
    }

    public boolean getIntakeUp() {
        return driver.getDPadUp();
    }

    public boolean getIntakeDown() {
        return driver.getDPadDown();
    }

    public boolean getToggleIntake() {
        return driver.getAButton();
    }

    public boolean getClimber() {
        return driver.getYButton();
    }

    public boolean getToggleIntakeClosed() {
        return driver.getDPadRight();
    }

}
