package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

public class Odometry {
    private Pose2d robotPose;
    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;

    private double left_previous;
    private double right_previous;
    private double perpen_previous;

    public Odometry (Pose2d initialPose, Rotation2d gyroInitialAngle) {
        this.robotPose = initialPose;
        this.gyroOffset = initialPose.getRotation().minus(gyroInitialAngle);
        left_previous = 0;
        right_previous = 0;
        perpen_previous = 0;
    }

    public Odometry (Pose2d initialPose, Rotation2d gyroInitialAngle, Rotation2d offset) {
        this.robotPose = initialPose;
        this.gyroOffset = offset;
        this.previousAngle = gyroInitialAngle;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public Pose2d update(double leftLocation, double rightLocation, double perpenLocation, Rotation2d currentIMU) {
        double delta_left = leftLocation - left_previous;
        double delta_right = rightLocation - right_previous;
        double delta_perpen = perpenLocation - perpen_previous;

        left_previous = leftLocation;
        right_previous = rightLocation;
        perpen_previous = perpenLocation;

        double centerMovement = (delta_left + delta_right)/2;

        Rotation2d angle = currentIMU.plus(gyroOffset);

        Pose2d position = robotPose.exp(new Twist2d(centerMovement, delta_perpen, angle.minus(previousAngle).getRadians()));

        previousAngle = angle;
        robotPose = position;

        return robotPose;
    }
}
