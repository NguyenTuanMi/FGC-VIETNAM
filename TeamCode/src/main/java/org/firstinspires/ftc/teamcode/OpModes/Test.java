package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;

@TeleOp(name = "Test")
public class Test extends OpMode {
//    private DcMotor testMotor;
//    private IMU imu;
    private Servo servo;
    private Climber climber;
    private Drivebase drivebase;
    private boolean atAngle = false;
    @Override
    public void init() {
        drivebase = new Drivebase(this);
        climber = new Climber(this);
        drivebase.init();
        climber.init();
//        imu = new IMU(this);
//        imu.init();
//        testMotor = hardwareMap.get(DcMotor.class, "test");
//        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        double angle = 0;
        if(gamepad1.square) {
            angle = 1;
        }
        else if(gamepad1.cross) {
            angle = -1;
        }
        climber.climb(angle);

//        if(atAngle) {
//            angle = 0.8;
//            servo.setPosition(angle);
//        }
//        telemetry.addData("Position y", imu.getPosition());
//        telemetry.update();
//        double speed = gamepad1.left_stick_y;
//        if (gamepad1.circle) {
//            testMotor.setPower(speed);
//        }
//        telemetry.addData("Speed", speed);
//        telemetry.update();
    }
}
