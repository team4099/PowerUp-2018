package org.usfirst.frc.team4099.robot.drive;

import org.usfirst.frc.team4099.lib.drive.DriveSignal;
import org.usfirst.frc.team4099.lib.joystick.JoystickUtils;
import org.usfirst.frc.team4099.lib.util.Utils;

/**
 * Curvature Drive
 * The left joystick controls speed (throttle)
 * The right joystick controls the curvature of the path.
 *
 * Credits: Team 254
 */

public class CDriveHelper {

    private static CDriveHelper sInstance = new CDriveHelper();

    private double mQuickStopAccumulator;

    private static final double kThrottleDeadband = 0.02;
    private static final double kWheelDeadband= 0.02;
    private static final double kTurnSensitivity = 2.0;

    private DriveSignal mSignal = new DriveSignal(0, 0);

    public static CDriveHelper getInstance() {
        return sInstance;
    }

    public DriveSignal curvatureDrive(double throttle, double wheel, boolean isQuickTurn) {
        throttle = JoystickUtils.deadband(throttle, kThrottleDeadband);
        wheel = -JoystickUtils.deadband(wheel, kWheelDeadband);

        double overPower;
        double angularPower;

        if (isQuickTurn) {
            if (Math.abs(throttle) < 0.2) {
                double alpha = 0.1;
                mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator +
                                        alpha * Utils.limit(wheel, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * kTurnSensitivity - mQuickStopAccumulator;

            if (mQuickStopAccumulator > 1) {
                mQuickStopAccumulator -= 1;
            } else if (mQuickStopAccumulator < -1) {
                mQuickStopAccumulator += 1;
            } else {
                mQuickStopAccumulator = 0.0;
            }
        }

        double rightPWM = throttle - angularPower;
        double leftPWM = throttle + angularPower;
        if (leftPWM > 1.0) {
            rightPWM -= overPower * (leftPWM - 1.0);
            leftPWM = 1.0;
        } else if (rightPWM > 1.0) {
            leftPWM -= overPower * (rightPWM - 1.0);
            rightPWM = 1.0;
        } else if (leftPWM < -1.0) {
            rightPWM += overPower * (-1.0 - leftPWM);
            leftPWM = -1.0;
        } else if (rightPWM < -1.0) {
            leftPWM += overPower * (-1.0 - rightPWM);
            rightPWM = -1.0;
        }

        mSignal.setRightMotor(rightPWM);
        mSignal.setLeftMotor(leftPWM);

        return mSignal;
    }
}
