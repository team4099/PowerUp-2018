package org.usfirst.frc.team4099.robot

class Constants {

    object Gains {
        val LEFT_KP = 0.0115
        val LEFT_KI = 0.0000
        val LEFT_KD = 0.0000
        val LEFT_KF = 0.0000

        val RIGHT_KP = 0.0115
        val RIGHT_KI = 0.0000
        val RIGHT_KD = 0.0000
        val RIGHT_KF = 0.0000
    }

    object Drive {
        val LEFT_MASTER_ID = 2
        val LEFT_SLAVE_1_ID = 3
        val LEFT_SLAVE_2_ID = 0
        val RIGHT_MASTER_ID = 4
        val RIGHT_SLAVE_1_ID = 5
        val RIGHT_SLAVE_2_ID = 0

        val SHIFTER_CHANNEL = 0
        val SHIFTER_MODULE = 0
    }

    object Joysticks {
        val DRIVER_PORT = 0
        val SHOTGUN_PORT = 1
    }

    object Loopers {
        val LOOPER_DT = 0.005 // 200 Hz
    }

    object Autonomous {
        val AUTO_OPTIONS_DASHBOARD_KEY = "auto_options"
        val SELECTED_AUTO_MODE_DASHBOARD_KEY = "selected_auto_mode"

        val CONNECTION_TIMEOUT_MILLIS = 1000
        val NUMBER_OF_TRIES = 5
    }
    object Arm {
        val MASTER_SRX_ID = 2
        val SLAVE_SRX_1_ID = 3
        val SLAVE_SRX_2_ID = 4
        val SLAVE_SRX_3_ID = 5

        val BRAKE_FORWARD_CHANNEL = 6
        val BRAKE_REVERSE_CHANNEL = 7

        val MIN_SPEED = -50
        val MAX_SPEED = 50

    }

    object Intake {
        val LEFT_INTAKE_TALON_ID = 0
        val RIGHT_INTAKE_TALON_ID = 1
    }

    object Wrist {
        val WRIST_TALON_ID = 6

        val KP = 0.0115
        val KI = 0.0000
        val KD = 0.0000
        val KF = 0.0000
    }

}
