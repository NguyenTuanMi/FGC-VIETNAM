package org.firstinspires.ftc.teamcode.VNRobot;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;

public class LocalizeRobot {
    private Drivebase drivebase;
    private IMU imu;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private FtcDashboard dashboard;
    private double[] x = new double[5];
    private double[] y = new double[5];
    private double[] theta = new double[5];
    public LocalizeRobot(OpMode opMode) {
        drivebase = new Drivebase(opMode);
        imu = new IMU(opMode);

        gamepad = opMode.gamepad1;
        telemetry = opMode.telemetry;
        dashboard = FtcDashboard.getInstance();

    }


    public void updateAngle(double heading) {
        for (int i = 0; i+1<theta.length; i++) {
            theta[i] = ANGLE_DIRECTIONS[i] + heading;
        }
    }

    public void updateCoordinate(double x_axis, double y_axis) {
        for (int i = 0; i<x.length; i++ ) {
            x[i] = x_axis - WHEEL_TRANSITIONS[i]*Math.cos(theta[i]);
            y[i] = y_axis - WHEEL_TRANSITIONS[i]*Math.sin(theta[i]);
        }
    }

    public void updateTelemetry(double heading, double x, double y) {
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", heading);
    }

    public IMU getIMU() {
        return imu;
    }

    public Drivebase getDrivebase() {
        return drivebase;
    }

    public void init() {
        drivebase.init();
        imu.init();
    }

    public void runOpMode() {

        TelemetryPacket packet = new TelemetryPacket();
        drivebase.setSpeed(-gamepad.right_stick_y*0.5, -gamepad.left_stick_y*0.5);
        telemetry.addData("Left position", drivebase.getLeftPosition());
        telemetry.addData("Right position", drivebase.getRightPosition());
        telemetry.update();

        packet.fieldOverlay().setFill("black").fillPolygon(x, y);
        dashboard.sendTelemetryPacket(packet);
    }
}