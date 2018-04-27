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
        val ELEVATOR_UP_KP = 0.6
        val ELEVATOR_UP_KI = 0.0008
        val ELEVATOR_UP_KD = 55.000
        val ELEVATOR_UP_KF = 0.5700

        val ELEVATOR_DOWN_KP = 1.00
        val ELEVATOR_DOWN_KI = 0.002
        val ELEVATOR_DOWN_KD = 60.0
        val ELEVATOR_DOWN_KF = 0.78

        //subject to change
        val WRIST_KP = 0.0000
        val WRIST_KI = 0.0000
        val WRIST_KD = 0.0000
        val WRIST_KF = 0.0000
    }

    object Drive {
        val LEFT_MASTER_ID = 4
        val LEFT_SLAVE_1_ID = 5
        val LEFT_SLAVE_2_ID = 6
        val RIGHT_MASTER_ID = 8
        val RIGHT_SLAVE_1_ID = 9
        val RIGHT_SLAVE_2_ID = 10

        val HIGH_GEAR_MAX_SETPOINT = 17.0  //17 fps

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

    object Dashboard {
        val ALLIANCE_COLOR_KEY = "dashboard/allianceColor"
        val ALLIANCE_OWNERSHIP_KEY = "dashboard/allianceOwnership"
    }

    object Autonomous {
        val AUTO_OPTIONS_DASHBOARD_KEY = "autonomous/autoOptions"
        val SELECTED_AUTO_MODE_DASHBOARD_KEY = "autonomous/selectedMode"

        val AUTO_STARTS_DASHBOARD_KEY = "autonomous/autoStarts"
        val SELECTED_AUTO_START_POS_KEY = "autonomous/selectedStart"

        val SELECTED_AUTO_START_DELAY_KEY = "autonomous/selectedDelay"

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
        val RIGHT_INTAKE_TALON_ID = 8
        val SHIFTER_FORWARD_ID = 6
        val SHIFTER_REVERSE_ID = 7

        val RIBBON_SWITCH_PORT = 9
    }

    object Elevator {
        val ELEVATOR_TALON_ID = 0
    }

    object Wrist {
        val WRIST_TALON_ID = 1

//        val WRIST_UP_KP = 1.0525
//        val WRIST_UP_KI = 0.0000
//        val WRIST_UP_KD = 0.000
//        val WRIST_UP_KF = 1.5547
//
//        val WRIST_DOWN_KP = 3.5460
//        val WRIST_DOWN_KI = 0.0456
//        val WRIST_DOWN_KD = 0.0000
//        val WRIST_DOWN_KF = 1.1440

        val WRIST_UP_KP = 1.0000
        val WRIST_UP_KI = 0.0050
        val WRIST_UP_KD = 2.0000
        val WRIST_UP_KF = 1.5547

        val WRIST_DOWN_KP = 0.1000
        val WRIST_DOWN_KI = 0.0100
        val WRIST_DOWN_KD = 1.0000
        val WRIST_DOWN_KF = 1.1440
    }

    object Forks {
        val LATCH_FORWARD_ID = 4
        val LATCH_REVERSE_ID = 5
    }

    object Climber {
        val CLIMBER_TALON_ID = 7
    }

}
