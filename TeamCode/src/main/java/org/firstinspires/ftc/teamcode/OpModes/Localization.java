package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VNRobot.LocalizeRobot;

@TeleOp(name = "Localization")
public class Localization extends LinearOpMode {
    private DifferentialDriveOdometry odometry;
    private LocalizeRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LocalizeRobot(this);
        robot.init();

        odometry = new DifferentialDriveOdometry(
                new Rotation2d(robot.getIMU().getYaw()),
                new Pose2d(0, 0, new Rotation2d(0)));

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), new Rotation2d(0));
            }
            odometry.update(
                    new Rotation2d(robot.getIMU().getYaw()),
                    robot.getDrivebase().getLeftPosition() / 300 * 0.09 * Math.PI,
                    robot.getDrivebase().getRightPosition() / 300 * 0.09 * Math.PI);
            robot.updateAngle(odometry.getPoseMeters().getHeading());
            robot.updateCoordinate(odometry.getPoseMeters().getX() * 39.3701, odometry.getPoseMeters().getY() * 39.3701);
            robot.updateTelemetry(
                    odometry.getPoseMeters().getHeading(),
                    odometry.getPoseMeters().getX() * 39.3701,
                    odometry.getPoseMeters().getY() * 39.3701);

            robot.runOpMode();
        }
    }
}
