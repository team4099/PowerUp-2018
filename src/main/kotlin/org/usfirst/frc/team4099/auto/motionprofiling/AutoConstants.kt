package org.usfirst.frc.team4099.auto.motionprofiling

class AutoConstants{
    val MAX_VELOCITY
    val MAX_ACCELERATION
    val MAX_JERK
    val WHEEL_BASE_LENGTH
    val WHEEL_BASE_WIDTH

    val LEFT_HIGH_KP = .1 * 1023 / 70
    val LEFT_HIGH_KI = 0.0000
    val LEFT_HIGH_KD = 0.0000
    val LEFT_HIGH_KF = 1023.0 / 4420.0

    val RIGHT_HIGH_KP = .1 * 1023 / 70
    val RIGHT_HIGH_KI = 0.0000
    val RIGHT_HIGH_KD = 0.0000
    val RIGHT_HIGH_KF = 1023.0 / 4420.0
}