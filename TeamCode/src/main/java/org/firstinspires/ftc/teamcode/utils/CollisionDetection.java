package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Constants;

public class CollisionDetection {
    private static boolean collisionDetected = false;
    private static double XpreAccel;
    private static double YpreAccel;

    public static void init(double XcurrAccel, double YcurrAccel) {
        XpreAccel = XcurrAccel;
        YpreAccel = YcurrAccel;
    }

    public void update(double XcurrAccel, double YcurrAccel) {
        double JerkX = XcurrAccel - XpreAccel;
        double JerkY = YcurrAccel - YpreAccel;

        XpreAccel = XcurrAccel;
        YpreAccel = YcurrAccel;

        if ((Math.abs(JerkX) > Constants.COLLISION.KD) || (Math.abs(JerkY) > Constants.COLLISION.KD)) {
            collisionDetected = true;
        } else if ((Math.abs(JerkX) < Constants.COLLISION.KD) && (Math.abs(JerkY) < Constants.COLLISION.KD)) {
            collisionDetected = false;
        }
    }

    public static boolean isCollided() {
        return collisionDetected;
    }
}
