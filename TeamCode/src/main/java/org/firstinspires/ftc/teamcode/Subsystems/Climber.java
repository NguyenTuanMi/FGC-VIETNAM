package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Climber {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private CRServo arm1;
    private CRServo arm2;
    private TouchSensor limit;
    private HardwareMap hardwareMap;

    public Climber(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "mr1");
        motor2 = hardwareMap.get(DcMotor.class, "mr2");
        motor3 = hardwareMap.get(DcMotor.class, "mr3");
        motor4 = hardwareMap.get(DcMotor.class, "mr4");
        arm1 = hardwareMap.get(CRServo.class, "arm1");
        arm2 = hardwareMap.get(CRServo.class, "arm2");

        limit = hardwareMap.get(TouchSensor.class, "limit");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        motor4.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        arm1.resetDeviceConfigurationForOpMode();
//        arm2.resetDeviceConfigurationForOpMode();

        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void climb(double speed) {
        motor1.setPower(speed);
        motor2.setPower(speed);
        motor3.setPower(speed);
        motor4.setPower(speed);
    }

    public void armController(double speed) {
        arm1.setPower(speed);
        arm2.setPower(speed);
    }

    public boolean getLimit() {
        return limit.isPressed();
    }

}
