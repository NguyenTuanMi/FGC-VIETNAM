package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.*;

public class ProjectileWithZone {
    private static double velocity = 0;
    private static double angle = 0;
    private static boolean Active = false;

    public enum ShooterPosition {
        ZONE_1, ZONE_2, ZONE_3, EMERGENCY
    }

    public static void activeWhen(boolean isActive) {
        Active = isActive;
    }

    public static ShooterPosition getShooterPose(double distance) {
        if (distance < ZONE_1) {
            return ShooterPosition.ZONE_1;
        }

        else if (distance < ZONE_2) {
            return ShooterPosition.ZONE_2;
        }

        else if (distance < ZONE_3) {
            return ShooterPosition.ZONE_3;
        }

        return ShooterPosition.EMERGENCY;
    }

    public static void update(ShooterPosition pose) {
        if(Active) {
            switch (pose) {
                case ZONE_1:
                    velocity = ZONE_1_SPEED;
                    angle = ZONE_1_ANGLE;
                case ZONE_2:
                    velocity = ZONE_2_SPEED;
                    angle = ZONE_2_ANGLE;
                case ZONE_3:
                    velocity = ZONE_3_SPEED;
                    angle = ZONE_3_ANGLE;
                case EMERGENCY:
                    velocity = EMERGENCY_SPEED;
                    angle = EMERGENCY_ANGLE;
            }
        }
    }

    public static double getVelocity() {
        return velocity;
    }

    public static double getAngle() {
        return angle;
    }
}
