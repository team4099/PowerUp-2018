package org.usfirst.frc.team4099.robot;

public class Constants {

    public class Gains {
        public static final double TURN_P = 0.0115;
        public static final double TURN_I = 0.0000;
        public static final double TURN_D = 0.0000;
        public static final double TURN_F = 0.0000;

        public static final double FORWARD_P = 0.0115;
        public static final double FORWARD_I = 0.0000;
        public static final double FORWARD_D = 0.0000;
        public static final double FORWARD_F = 0.0000;

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

        public static final double LEFT_ENCODER_INCHES_PER_PULSE = 1;
        public static final double RIGHT_ENCODER_INCHES_PER_PULSE = 1;

        public static final double TURN_TOLERANCE_DEGREES = 2;
        public static final double FORWARD_TOLERANCE_INCHES = 3;

        public static final double AUTO_TURN_MAX_POWER = .35;
        public static final double AUTO_FORWARD_MAX_POWER = .5;

        public static final int ENCODER_SAMPLES_TO_AVERAGE = 10;

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

    public class Autonomous {
        public static final String UDOO_ADDRESS = "http://10.40.99.10/";

        public static final double BACK_OUT_INCHES = -40;
        public static final double AIRSHIP_WIDTH_INCHES = 100;
        public static final double DISTANCE_PAST_AIRSHIP_INCHES = 60;

        public static final double EXTRA_INCHES = 5;

        public static final double WAIT_TIME_ON_LIFT = 1;
    }

}
