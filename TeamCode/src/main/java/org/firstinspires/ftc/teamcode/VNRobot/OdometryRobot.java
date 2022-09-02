package org.firstinspires.ftc.teamcode.VNRobot;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.INIT_THETA;
import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.INIT_X;
import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.INIT_Y;
import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.HOOD_X;
import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.HOOD_Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.utils.Map;
import org.firstinspires.ftc.teamcode.utils.Odometry;

public class OdometryRobot {
    private Odometry odometry;
    private final Drivebase drivebase;
    private final IMU imu;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final FtcDashboard dashboard;
    private TelemetryPacket packet;
    private boolean rotateMode = false;

    public OdometryRobot(OpMode opMode) {
        telemetry = opMode.telemetry;
        gamepad = opMode.gamepad1;

        drivebase = new Drivebase(opMode);
        imu = new IMU(opMode);
        dashboard = FtcDashboard.getInstance();
    }

    public void init() {
        drivebase.init();
        imu.init();
    }

    public void start() {
        odometry = new Odometry(
                new Pose2d(INIT_X, INIT_Y, new Rotation2d(INIT_THETA)),
                new Rotation2d(imu.getYaw())
        );
    }


    public void loop() {
        packet = new TelemetryPacket();
        double left = gamepad.left_stick_y * 0.5;
        double right = gamepad.right_stick_y * 0.5;
        double leftPosition = drivebase.getLeftPosition() / 300 * 0.09 * Math.PI;
        double rightPosition = drivebase.getRightPosition() / 300 * 0.09 * Math.PI;
        double perpenPosition = 0;



        odometry.update(leftPosition, rightPosition, perpenPosition, new Rotation2d(imu.getYaw()));

        if(gamepad.cross) {
            rotateMode = true;
        }
        else if(gamepad.square) {
            rotateMode = false;
        }

        if(rotateMode) {
            double setPoint = Math.atan((odometry.getRobotPose().getY() - HOOD_Y) / (odometry.getRobotPose().getX() - HOOD_X));
            left = drivebase.rotateAngle(imu.getYaw(), setPoint);
            right = -drivebase.rotateAngle(imu.getYaw(), setPoint);
        }

        drivebase.tankController(right, left);

        Map.updateAngle(odometry.getRobotPose().getHeading());
        Map.updateCoordinate(odometry.getRobotPose().getX() * 39.3701, odometry.getRobotPose().getY() * 39.3701);

        telemetry.addData("Left position", leftPosition);
        telemetry.addData("Right position", rightPosition);
        telemetry.addData("Perpendicular position", perpenPosition);
        telemetry.addData("X", odometry.getRobotPose().getX());
        telemetry.addData("y", odometry.getRobotPose().getY());
        telemetry.addData("Theta", odometry.getRobotPose().getHeading());
        telemetry.addData("Right stick", gamepad.right_stick_y);
        telemetry.update();

        packet.fieldOverlay()
                .setFill("black")
                .fillPolygon(Map.getX(), Map.getY());
        dashboard.sendTelemetryPacket(packet);
    }
}
