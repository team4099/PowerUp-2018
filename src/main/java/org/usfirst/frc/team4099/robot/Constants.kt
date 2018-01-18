package org.usfirst.frc.team4099.robot

class Constants {

    object Gains {
        val LEFT_LOW_KP = 0.0115
        val LEFT_LOW_KI = 0.0000
        val LEFT_LOW_KD = 0.0000
        val LEFT_LOW_KF = 0.0000

        //subject to change
        val LEFT_HIGH_KP = 0.0115
        val LEFT_HIGH_KI = 0.0000
        val LEFT_HIGH_KD = 0.0000
        val LEFT_HIGH_KF = 0.0000

        val RIGHT_LOW_KP = 0.0115
        val RIGHT_LOW_KI = 0.0000
        val RIGHT_LOW_KD = 0.0000
        val RIGHT_LOW_KF = 0.0000

        //subject to change
        val RIGHT_HIGH_KP = 0.0115
        val RIGHT_HIGH_KI = 0.0000
        val RIGHT_HIGH_KD = 0.0000
        val RIGHT_HIGH_KF = 0.0000

        //subject to change
    }

    object Drive {
        val LEFT_MASTER_ID = 2
        val LEFT_SLAVE_1_ID = 3
        val LEFT_SLAVE_2_ID = 0
        val RIGHT_MASTER_ID = 4
        val RIGHT_SLAVE_1_ID = 5
        val RIGHT_SLAVE_2_ID = 0

        val HIGH_GEAR_MAX_SETPOINT = 17.0  //17 fps
    }

    object Wheels {
        val DRIVE_WHEEL_DIAMETER_INCHES = 0
    }

    object Velocity {
        val HIGH_GEAR_VELOCITY_CONTROL_SLOT = 0
        val LOW_GEAR_VELOCITY_CONTROL_SLOT = 0
        val DRIVE_HIGH_GEAR_NOMINAL_OUTPUT = 0.0 //percentage
        val DRIVE_LOW_GEAR_NOMINAL_OUTPUT = 0.0 //percentage
        val DRIVE_HIGH_GEAR_FORWARD_OUTPUT = 1.0 //percentage
        val DRIVE_HIGH_GEAR_REVERSE_OUTPUT = -1.0
        val DRIVE_LOW_GEAR_FORWARD_OUTPUT = 1.0
        val DRIVE_LOW_GEAR_REVERSE_OUTPUT = -1.0
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

}
