package org.firstinspires.ftc.teamcode.VNRobot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Odometry;
import org.firstinspires.ftc.teamcode.utils.ProjectileWithZone;
import org.firstinspires.ftc.teamcode.utils.ProjectileWithoutZone;

import static org.firstinspires.ftc.teamcode.Constants.SHOOT.*;
import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.*;
import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;

public class FunctionalRobot {
    private IMU imu;
    private Drivebase drivebase;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private Extension extension;
    private Intake sucker;
    private Shooter shooter;
    private Hood hood;
    private PIDController shootPID;
    private Odometry odometry;
    private boolean activeShooter = false;
    private boolean rotateMode = false;
    private double initAngle = 0;

    public FunctionalRobot(OpMode opMode) {
        imu = new IMU(opMode);
        drivebase = new Drivebase(opMode);
        sucker = new Intake(opMode);
        extension = new Extension(opMode);
        shooter = new Shooter(opMode);
        hood = new Hood(opMode);
        shootPID = new PIDController(KP, KI, KD);

        telemetry = opMode.telemetry;
        gamepad = opMode.gamepad1;
    }

    public void init() {
        imu.init();
        drivebase.init();
        extension.init();
        sucker.init();
        shooter.init();
        hood.init();

        odometry = new Odometry(
                        new Pose2d(new Translation2d(INIT_X, INIT_Y), new Rotation2d(INIT_THETA)),
                        new Rotation2d(imu.getYaw()));

        shootPID.setIntegrationBounds(MIN_INTERGRATOR, MAX_INTERGRATOR);
        shootPID.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    }

    public void runOpMode() {
        double left = -gamepad.left_stick_y;
        double right = -gamepad.right_stick_y;
        double intake = 0;
        double extend = 0;
        double distance = 0;

        odometry.update(drivebase.getLeftPosition(), drivebase.getRightPosition(), 0, new Rotation2d(imu.getYaw()));

        distance = Math.hypot(odometry.getRobotPose().getX() - HOOD_X, odometry.getRobotPose().getY() - HOOD_Y) - HOOD_RADIUS;

        if(gamepad.triangle) {
            intake = 1;
        }

        if(gamepad.circle) {
            extend = 1;
        }

        if(gamepad.square) {
            activeShooter = true;
        }
//        else if (gamepad.cross) {
//            activeShooter = false;
//        }

        if(gamepad.cross) {
            rotateMode = true;
        }

        if(gamepad.left_bumper) {
            intake = -intake;
            extend = -extend;
            activeShooter = false;
            rotateMode = false;
        }


//        else if(gamepad.dpad_up) {
//            rotateMode = false;
//        }

        if (activeShooter) {
            ProjectileWithoutZone.update(distance, HOOD_HEIGHT);
            ProjectileWithoutZone.calculate();
            double angleRate = ProjectileWithoutZone.getAngleRate();
            double theta = ProjectileWithoutZone.getAngle();
//            shooter.shoot(angleRate);
            hood.setPosition(theta - initAngle);
            if(hood.hasReached(theta - initAngle)) {
                shooter.shoot(angleRate);
            }
        }

        if (rotateMode) {
            double setPoint = Math.atan((odometry.getRobotPose().getY()-HOOD_Y)/ (odometry.getRobotPose().getX()-HOOD_X));
            left = drivebase.rotateAngle(imu.getYaw(), setPoint);
            right = -drivebase.rotateAngle(imu.getYaw(), setPoint);
        }


//        ProjectileWithZone.update(ProjectileWithZone.getShooterPose(distance));

//        double angleRate = ProjectileWithZone.getVelocity();
//        double theta = ProjectileWithZone.getAngle();


        drivebase.setSpeed(left, right);
        sucker.setSpeed(intake);
        extension.extend(extend);

        //double shoot = shootPID.calculate(shooter.getVelocity(), angleRate);

        telemetry.addData("Shooter is calibrating", activeShooter);
        telemetry.addData("Shooter current velocity",shooter.getVelocity());
        telemetry.addData("Hood curent position", hood.getPosition());
        telemetry.update();
    }
}
