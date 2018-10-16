package org.usfirst.frc.team4099.auto.motionprofiling





object AutoConstants{
    val MAX_VELOCITY = 17.0
    val MAX_ACCELERATION = 5.0
    val MAX_JERK = 5.0
    val WHEEL_BASE_LENGTH = 3.0
    val WHEEL_BASE_WIDTH = 10.0

    val LEFT_HIGH_KP = .1 * 1023 / 70
    val LEFT_HIGH_KI = 0.0000
    val LEFT_HIGH_KD = 0.0000
    val LEFT_HIGH_KF = 1023.0 / 4420.0

    val RIGHT_HIGH_KP = .1 * 1023 / 70
    val RIGHT_HIGH_KI = 0.0000
    val RIGHT_HIGH_KD = 0.0000
    val RIGHT_HIGH_KF = 1023.0 / 4420.0
}