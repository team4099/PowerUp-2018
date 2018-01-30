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

        val SHIFTER_CHANNEL = 0
        val SHIFTER_MODULE = 0
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

    object Wheels {
        val TRACK_SCRUB_FACTOR = 0.924 //todo
        val TRACK_WIDTH_INCHES = 26.655 //todo
        val DRIVE_WHEEL_DIAMETER_INCHES = 0 //todo
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

    object PathFollowing { //todo all of these constants
        val PATH_FOLLOWING_MAX_ACCEL = 120.0
        val SEGMENT_COMPLETION_TOLERANCE = 0.1
        val PATH_STOP_STEERING_DIST: Double = 9.0
        val PATH_FOLLOWING_GOAL_VEL_TOL: Double = 12.0
        val PATH_FOLLOWING_GOAL_POS_TOL: Double = 0.75
        val PATH_FOLLOWING_MAX_VEL: Double = 120.0
        val PF_Kffa: Double = 0.05
        val PF_Kffv: Double = 1.0
        val PF_Kv: Double = 0.02
        val PF_Ki: Double = 0.03
        val PF_Kp: Double = 5.0
        val INERTIA_STEERING_GAIN: Double = 0.0
        val MAX_LOOK_AHEAD_SPEED: Double = 120.0
        val MIN_LOOK_AHEAD_SPEED: Double = 9.0
        val MAX_LOOK_AHEAD: Double = 24.0
        val MIN_LOOK_AHEAD: Double = 12.0
    }

    object Intake {
        val LEFT_INTAKE_TALON_ID = 0
        val RIGHT_INTAKE_TALON_ID = 1
    }

}
