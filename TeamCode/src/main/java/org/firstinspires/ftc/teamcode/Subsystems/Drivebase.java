package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.ROTATION.*;


public class Drivebase {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private HardwareMap hardwareMap;
    private PIDController controller;

    public Drivebase(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.controller = new PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    }

    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "lm");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rm");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller.setTolerance(TOLERANCE);
        controller.setIntegrationBounds(INTERGRAL_MIN, INTERGRAL_MAX);
    }

    public void setSpeed(double right, double left) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    public int getLeftPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightMotor.getCurrentPosition();
    }

    public double rotateAngle(double angle, double setPoint) {
        controller.setSetPoint(setPoint);
        return controller.calculate(angle);
    }

    public boolean atSetpoint() {
        return controller.atSetPoint();
    }
}
