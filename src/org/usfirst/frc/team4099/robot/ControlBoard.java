package org.usfirst.frc.team4099.robot;

import org.usfirst.frc.team4099.lib.joystick.DualShock4Gamepad;

public class ControlBoard {
    private static ControlBoard sInstance = new ControlBoard();
    public static ControlBoard getInstance() {
        return sInstance;
    }
    private final DualShock4Gamepad driver;

    private ControlBoard() {
        driver = new DualShock4Gamepad(Constants.Joysticks.DRIVER_PORT);
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
