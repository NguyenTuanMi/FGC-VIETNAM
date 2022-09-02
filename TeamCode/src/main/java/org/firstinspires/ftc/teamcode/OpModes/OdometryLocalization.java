package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VNRobot.OdometryRobot;

@TeleOp(name = "Odometry")
public class OdometryLocalization extends OpMode {
    private OdometryRobot robot;

    @Override
    public void init() {
        robot = new OdometryRobot(this);
        robot.init();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop();
    }
}
