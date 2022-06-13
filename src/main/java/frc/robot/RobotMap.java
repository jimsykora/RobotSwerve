
package frc.robot;

public final class RobotMap {

    public static final class map_Controllers {

        public static final int DRIVER = 0;
    }

    public static final class map_Drivetrain {

        public static final int FRONT_LEFT_DRIVE = 1; //the number = the CAN order
        public static final int FRONT_LEFT_TURN = 0;

        public static final int FRONT_RIGHT_DRIVE = 7;
        public static final int FRONT_RIGHT_TURN = 6;

        public static final int BACK_LEFT_DRIVE = 3;
        public static final int BACK_LEFT_TURN = 2;

        public static final int BACK_RIGHT_DRIVE = 5;
        public static final int BACK_RIGHT_TURN = 4;

        public static final int FRONT_LEFT_TURN_ENCODER_A = 0;
        public static final int FRONT_LEFT_TURN_ENCODER_B = 1;

        public static final int FRONT_RIGHT_TURN_ENCODER_A = 2;
        public static final int FRONT_RIGHT_TURN_ENCODER_B = 3;

        public static final int BACK_LEFT_TURN_ENCODER_A = 4;
        public static final int BACK_LEFT_TURN_ENCODER_B = 5;

        public static final int BACK_RIGHT_TURN_ENCODER_A = 6;
        public static final int BACK_RIGHT_TURN_ENCODER_B = 7;
        
        public static final int FRONT_LEFT_STEER_ABS_ENCODER = 0;
        public static final int BACK_LEFT_STEER_ABS_ENCODER = 1;
        public static final int BACK_RIGHT_STEER_ABS_ENCODER = 2;
        public static final int FRONT_RIGHT_STEER_ABS_ENCODER = 3;
    }

}
