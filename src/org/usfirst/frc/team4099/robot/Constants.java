package org.usfirst.frc.team4099.robot;

public class Constants {

    public class Gains {

    }

    public class Drive {
        public static final int LEFT_FRONT_ID = 2;
        public static final int RIGHT_FRONT_ID = 0;
        public static final int LEFT_BACK_ID = 3;
        public static final int RIGHT_BACK_ID = 1;

        public static final int LEFT_ENCODER_A = 0;
        public static final int LEFT_ENCODER_B = 1;
        public static final int RIGHT_ENCODER_A = 2;
        public static final int RIGHT_ENCODER_B = 3;

        public static final double LEFT_ENCODER_DISTANCE_PER_PULSE = 1;
        public static final double RIGHT_ENCODER_DISTANCE_PER_PULSE = 1;

        public static final double TURN_TOLERANCE_DEGREES = 3;
        public static final double FORWARD_TOLERANCE_METERS = .05;

        public static final double TURN_MAX_POWER = .35;
        public static final double FORWARD_MAX_POWER = .5;

    }

    public class Joysticks {
        public static final int DRIVER_PORT = 0;
        public static final int SHOTGUN_PORT = 1;
    }

    public class Loopers {
        public static final double LOOPER_DT = 0.005; // 200 Hz
    }

    public class Climber {
        public static final int CLIMBER_TALON_ID = 4;
    }

    public class Intake {
        public static final int UP_DOWN_SOLENOID_FORWARD = 1;
        public static final int UP_DOWN_SOLENOID_REVERSE = 0;
        public static final int GRAB_SOLENOID_FORWARD = 3;
        public static final int GRAB_SOLENOID_REVERSE = 2;
    }

    public class Vision {
        public static final String UDOO_ADDRESS = "http://10.40.99.10/";
    }

}
