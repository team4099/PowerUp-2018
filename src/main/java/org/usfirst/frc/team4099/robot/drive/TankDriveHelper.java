package org.usfirst.frc.team4099.robot.drive;

import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils;

/**
 * Tank Drive
 * Left joystick controls the left motor speed.
 * Right joystick controls the right motor speed.
 */

public class TankDriveHelper {

    private static TankDriveHelper sInstance = new TankDriveHelper();
    private static final double kLeftDeadband = 0.05;
    private static final double kRightDeadband = 0.05;

    private DriveSignal mSignal = new DriveSignal(0, 0);

    public static TankDriveHelper getInstance() {
        return sInstance;
    }

    public DriveSignal tankDrive(double left, double right) {
        left = JoystickUtils.deadband(left, kLeftDeadband);
        right = JoystickUtils.deadband(right, kRightDeadband);

        mSignal.setLeftMotor(left);
        mSignal.setRightMotor(right);

        return mSignal;
    }

}
