package org.usfirst.frc.team4099.robot

class Constants {

    object Gains {
        val TURN_P = 0.0115
        val TURN_I = 0.0000
        val TURN_D = 0.0000
        val TURN_F = 0.0000

        val FORWARD_P = 0.0115
        val FORWARD_I = 0.0000
        val FORWARD_D = 0.0000
        val FORWARD_F = 0.0000

    }

    object Drive {
        val LEFT_FRONT_ID = 2
        val LEFT_BACK_ID = 3
        val RIGHT_FRONT_ID = 4
        val RIGHT_BACK_ID = 5

        val LEFT_ENCODER_A = 2
        val LEFT_ENCODER_B = 3
        val RIGHT_ENCODER_A = 0
        val RIGHT_ENCODER_B = 1

        val LEFT_ENCODER_INCHES_PER_PULSE = 0.0942408377
        val RIGHT_ENCODER_INCHES_PER_PULSE = 0.08612440191

        val TURN_TOLERANCE_DEGREES = 2.0
        val FORWARD_TOLERANCE_INCHES = 3.0

        val AUTO_TURN_MAX_POWER = .45
        val AUTO_FORWARD_MAX_POWER = .35

        val AUTO_FORWARD_SLOW_POWER = .25

        val ENCODER_SAMPLES_TO_AVERAGE = 10

        val FORWARD_KP = 0.07

    }

    object Joysticks {
        val DRIVER_PORT = 0
        val SHOTGUN_PORT = 1
    }

    object Loopers {
        val LOOPER_DT = 0.005 // 200 Hz
    }

    object Climber {
        val CLIMBER_TALON_ID = 9
    }

    object Intake {
        val UP_DOWN_SOLENOID_FORWARD = 0
        val UP_DOWN_SOLENOID_REVERSE = 1
        val GRAB_SOLENOID_FORWARD = 2
        val GRAB_SOLENOID_REVERSE = 3
    }

    object Autonomous {
        val UDOO_ADDRESS = "http://10.40.99.10:5800/"
        val CONNECTION_TIMEOUT_MILLIS = 1000
        val NUMBER_OF_TRIES = 5

        val BACK_OUT_INCHES = -40.0
        val AIRSHIP_WIDTH_INCHES = 100.0
        val DISTANCE_PAST_AIRSHIP_INCHES = 60.0

        val EXTRA_INCHES = 5.0

        val WAIT_TIME_ON_LIFT = 1.0
    }

}
