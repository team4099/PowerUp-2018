package org.usfirst.frc.team4099.robot;

import org.usfirst.frc.team4099.lib.joystick.LogitechF310Gamepad;

public class ControlBoard {
    private static ControlBoard sInstance = new ControlBoard();
    public static ControlBoard getInstance() {
        return sInstance;
    }
    private final LogitechF310Gamepad driver;
//    private final LogitechF310Gamepad operator;

    private ControlBoard() {
        driver = new LogitechF310Gamepad(Constants.Joysticks.DRIVER_PORT);
//        operator = new LogitechF310Gamepad(Constants.Joysticks.SHOTGUN_PORT);
    }

    public double getThrottle() {
        return driver.getLeftYAxis();
    }

    public double getTurn() {
        return driver.getRightXAxis();
    }

    /**
     * Should the bot drive in quick turn mode?
     * @return  true/false, depending on if the joystick is depressed
     */
    public boolean getQuickTurn() {
        return driver.getLeftShoulderButton();
    }

    public boolean getToggleIntakeUp() {
        return driver.getBButton();
    }

    public boolean getToggleIntakeGrab() {
        return driver.getAButton();
    }

    public boolean getClimber() {
        return driver.getYButton();
    }
}
