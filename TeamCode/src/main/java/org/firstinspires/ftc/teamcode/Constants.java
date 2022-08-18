package org.firstinspires.ftc.teamcode;


// All calculation must be performed in SI units besides odometry
// For measuring the angle, we must use the
public final class Constants {
    public static final class ODOMETRY {
        public static final double INIT_X = 0;
        public static final double INIT_Y = 0;
        public static final double INIT_THETA = 0;

        public static final double ROBOT_WIDTH = 0.45*39.3701;
        public static final double ROBOT_LENGTH = 0.48*39.3701;
        public static final double ARROW_LENGTH = 2;

        // lf, lb, rf, rb, tracking arrow
        public static final double[] ANGLE_DIRECTIONS =
                {
                        Math.atan2(ROBOT_WIDTH,ROBOT_LENGTH),
                        Math.atan2(ROBOT_WIDTH,ROBOT_LENGTH) + Math.PI/2,
                        -Math.atan2(ROBOT_WIDTH,ROBOT_LENGTH),
                        -( Math.atan2(ROBOT_WIDTH,ROBOT_LENGTH) + Math.PI/2 ),
                        0
                };
        public static final double[] WHEEL_TRANSITIONS =
                {
                        Math.hypot(ROBOT_WIDTH /2, ROBOT_LENGTH /2),
                        Math.hypot(ROBOT_WIDTH /2, ROBOT_LENGTH /2),
                        Math.hypot(ROBOT_WIDTH /2, ROBOT_LENGTH /2),
                        Math.hypot(ROBOT_WIDTH /2, ROBOT_LENGTH /2),
                        ARROW_LENGTH
                };
    }

    public static final class SHOOT {
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double MAX_INTERGRATOR = 0;
        public static final double MIN_INTERGRATOR = 0;
        public static final double VELOCITY_TOLERANCE = 0;
        public static final double POSITION_TOLERANCE = 0;
        public static final double HOOD_TOLERANCE = 0.01;
    }

    public static final class COLLISION {
        public static final double KD = 0.5f;
    }

    public static final class ROTATION {
        public static final double ROTATE_KP = 0.5;
        public static final double ROTATE_KI = 0;
        public static final double ROTATE_KD = 0;
        public static final double TOLERANCE = 0.05;
        public static final double INTERGRAL_MIN = 0;
        public static final double INTERGRAL_MAX = 1;
    }

    public static final class PROJECTILE_MOTION {
        public static final double ANGLE_RATE_ERROR = 0;
        public static final double THETA_ERROR = 0;
        public static final double R1 = 0.045;
        public static final double R2 = 0.03;
        public static final double GRAVITATIONAL_ACCEL = 9.81;
        public static final double HOOD_HEIGHT = 2.5;
        public static final double HOOD_X = 3.0023;
        public static final double HOOD_Y = 3.5;
        public static final double HOOD_RADIUS = 2*Math.sqrt(3)/5;

        public static final double ZONE_1 = 0;
        public static final double ZONE_2 = 0;
        public static final double ZONE_3 = 0;
        public static final double EMERGENCY = 0;

        public static final double ZONE_1_SPEED = 0;
        public static final double ZONE_2_SPEED = 0;
        public static final double ZONE_3_SPEED = 0;
        public static final double EMERGENCY_SPEED = 0;

        public static final double ZONE_1_ANGLE = 0;
        public static final double ZONE_2_ANGLE = 0;
        public static final double ZONE_3_ANGLE = 0;
        public static final double EMERGENCY_ANGLE = 0;

    }
}
