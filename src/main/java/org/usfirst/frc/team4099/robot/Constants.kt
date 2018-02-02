package org.usfirst.frc.team4099.robot

class Constants {

    object Gains {
        val LEFT_LOW_KP = 0.0000
        val LEFT_LOW_KI = 0.0000
        val LEFT_LOW_KD = 0.0000
        val LEFT_LOW_KF = 1023.0 / 2220.0

        //subject to change
        val LEFT_HIGH_KP = .1 * 1023 / 70
        val LEFT_HIGH_KI = 0.0000
        val LEFT_HIGH_KD = 0.0000
        val LEFT_HIGH_KF = 1023.0 / 4420.0

        val RIGHT_LOW_KP = 0.0000
        val RIGHT_LOW_KI = 0.0000
        val RIGHT_LOW_KD = 0.0000
        val RIGHT_LOW_KF = 1023.0 / 2220.0

        //subject to change
        val RIGHT_HIGH_KP = .1 * 1023 / 70
        val RIGHT_HIGH_KI = 0.0000
        val RIGHT_HIGH_KD = 0.0000
        val RIGHT_HIGH_KF = 1023.0 / 4420.0

        //subject to change
    }

    object Drive {
        val LEFT_MASTER_ID = 8
        val LEFT_SLAVE_1_ID = 6
        val LEFT_SLAVE_2_ID = 7
        val RIGHT_MASTER_ID = 1
        val RIGHT_SLAVE_1_ID = 2
        val RIGHT_SLAVE_2_ID = 3

        val HIGH_GEAR_MAX_SETPOINT = 17.0  //17 fps
        val SHIFTER_CHANNEL = 0
        val SHIFTER_MODULE = 0


        val SHIFTER_FORWARD_ID = 0
        val SHIFTER_REVERSE_ID = 1

    }

    object Wheels {
        val DRIVE_WHEEL_DIAMETER_INCHES = 0
    }

    object Velocity {
        val HIGH_GEAR_VELOCITY_CONTROL_SLOT = 0
        val LOW_GEAR_VELOCITY_CONTROL_SLOT = 0
        val DRIVE_HIGH_GEAR_NOMINAL_OUTPUT = 0.0 //percentage
        val DRIVE_LOW_GEAR_NOMINAL_OUTPUT = 0.0 //percentage
        val DRIVE_HIGH_GEAR_MAX_FORWARD_OUTPUT = 1.0 //percentage
        val DRIVE_HIGH_GEAR_MAX_REVERSE_OUTPUT = -1.0
        val DRIVE_LOW_GEAR_MAX_FORWARD_OUTPUT = 1.0
        val DRIVE_LOW_GEAR_MAX_REVERSE_OUTPUT = -1.0

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
        val LEFT_INTAKE_TALON_ID = 9
        val RIGHT_INTAKE_TALON_ID = 4
    }

    object Climber{
        val CLIMBER_TALON_ID=2
    }

}
