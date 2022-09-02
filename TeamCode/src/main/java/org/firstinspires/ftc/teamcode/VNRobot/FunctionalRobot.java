package org.firstinspires.ftc.teamcode.VNRobot;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Odometry;

public class FunctionalRobot {
    private final IMU imu;
    private final Drivebase drivebase;
    private final Climber climber;
    private final Shooter shooter;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private Odometry odometry;
    private int state;
    private boolean activateBoth = true;

    public FunctionalRobot(OpMode opMode) {
        imu = new IMU(opMode);
        drivebase = new Drivebase(opMode);
        climber = new Climber(opMode);
        shooter = new Shooter(opMode);
        telemetry = opMode.telemetry;
        gamepad = opMode.gamepad1;
    }

    public void init() {
        imu.init();
        drivebase.init();
        climber.init();
        shooter.init();

        odometry = new Odometry(
                new Pose2d(new Translation2d(INIT_X, INIT_Y), new Rotation2d(INIT_THETA)),
                new Rotation2d(imu.getYaw())
        );
    }

    public void runOpMode() {
        double left = -gamepad.left_stick_y;
        double right = -gamepad.right_stick_y;
//        double forward = -gamepad.left_stick_y;
//        double rotation = -gamepad.right_stick_x;
        double climb = 0;
        double arm = 0;
        double shoot = 0;
        boolean isLeftBumper = gamepad.left_bumper;
        boolean isSquare = gamepad.square;

        odometry.update(drivebase.getLeftPosition(), drivebase.getRightPosition(), 0, new Rotation2d(imu.getYaw()));

        if(gamepad.triangle) {
            arm = 1;
        }

        if(gamepad.circle) {
            shoot = 1;
        }

        if(activateBoth) {
            if(isLeftBumper) {
                state = 1; // down limit
                climb = -1;
                arm = -1;
                shoot = -1;
            }
            else if(isSquare) {
                state = -1; // up limit
                climb = 1;
            }

            if((state == 1 || state == -1) && climber.getLimit()) {
                climb = 0;
                activateBoth = false;
            }
        }

        else {
            if(state == -1) {
                if (isLeftBumper) {
                    climb = -1;
                    arm = -1;
                    shoot = -1;
                }
            }
            else if(state == 1) {
                if(isSquare) {
                    climb = 1;
                }
            }
            if(!climber.getLimit()) {
                activateBoth = true;
            }
        }

        telemetry.addData("Limit switch state", climber.getLimit());
        telemetry.addData("Current state", state);
        telemetry.addData("X position", odometry.getRobotPose().getX());
        telemetry.addData("Y position", odometry.getRobotPose().getY());
        telemetry.addData("Heading", odometry.getRobotPose().getHeading());
        telemetry.addData("Joystock right", -gamepad.right_stick_y);
        telemetry.addData("Joystick left value", left);
        drivebase.tankController(right, left);
//        drivebase.arcadeController(forward, rotation);
        climber.climb(climb);
        climber.armController(arm);
        shooter.shoot(shoot);

        telemetry.update();
    }
}
