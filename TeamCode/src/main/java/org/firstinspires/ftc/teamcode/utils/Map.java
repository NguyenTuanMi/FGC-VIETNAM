package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.ANGLE_DIRECTIONS;
import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.WHEEL_TRANSITIONS;

public class Map {
    private static final double[] x = new double[5];
    private static final double[] y = new double[5];
    private static final double[] theta = new double[5];

    public static void updateAngle(double heading) {
        for (int i = 0; i + 1 < theta.length; i++) {
            theta[i] = ANGLE_DIRECTIONS[i] + heading;
        }
    }

    public static void updateCoordinate(double x_axis, double y_axis) {
        for (int i = 0; i < x.length; i++) {
            x[i] = x_axis + WHEEL_TRANSITIONS[i] * Math.cos(theta[i]);
            y[i] = y_axis + WHEEL_TRANSITIONS[i] * Math.sin(theta[i]);
        }
    }

    public static double[] getX() {
        return x;
    }

    public static double[] getY() {
        return y;
    }

    public static double[] getTheta() {
        return theta;
    }
}
