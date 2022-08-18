package org.firstinspires.ftc.teamcode.VNRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Map;
import org.firstinspires.ftc.teamcode.utils.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;

public class OdometryRobot {
    private Odometry odometry;
    private Drivebase drivebase;
    private IMU imu;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

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
        odometry = new Odometry(
                new Pose2d(INIT_X, INIT_Y, new Rotation2d(INIT_THETA)),
                new Rotation2d(imu.getYaw())
        );
    }

    public void loop() {
        packet = new TelemetryPacket();
        double left = gamepad.left_stick_y;
        double right = gamepad.right_stick_y;
        double leftPosition = drivebase.getLeftPosition()/300*0.09*Math.PI;
        double rightPosition = drivebase.getRightPosition()/300*0.09*Math.PI;
        double perpenPosition = 0;

        drivebase.setSpeed(right, left);

        odometry.update(leftPosition, rightPosition, perpenPosition, new Rotation2d(imu.getYaw()));

        Map.updateAngle(odometry.getRobotPose().getHeading());
        Map.updateCoordinate(odometry.getRobotPose().getX()*39.3701, odometry.getRobotPose().getY()*39.3701);

        telemetry.addData("Left position", leftPosition);
        telemetry.addData("Right position", rightPosition);
        telemetry.addData("Perpendicular position", perpenPosition);
        telemetry.addData("X", odometry.getRobotPose().getX());
        telemetry.addData("y", odometry.getRobotPose().getY());
        telemetry.addData("Theta", odometry.getRobotPose().getHeading());
        telemetry.update();

        packet.fieldOverlay()
                .setFill("black")
                .fillPolygon(Map.getX(), Map.getY());
        dashboard.sendTelemetryPacket(packet);
    }
}
