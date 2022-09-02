package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.ROTATION.INTERGRAL_MAX;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.INTERGRAL_MIN;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.ROTATE_KD;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.ROTATE_KI;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.ROTATE_KP;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.TOLERANCE;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivebase {
    private DcMotorEx leftMaster;
    private DcMotorEx rightMaster;
    private DcMotorEx leftFollow;
    private DcMotorEx rightFollow;
    private final HardwareMap hardwareMap;
    private final PIDController controller;

    public Drivebase(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.controller = new PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    }

    public void init() {
        leftMaster = hardwareMap.get(DcMotorEx.class, "lm");
        rightMaster = hardwareMap.get(DcMotorEx.class, "rm");
        leftFollow = hardwareMap.get(DcMotorEx.class, "lf");
        rightFollow = hardwareMap.get(DcMotorEx.class, "rf");

        leftMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFollow.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMaster.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFollow.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFollow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFollow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller.setTolerance(TOLERANCE);
        controller.setIntegrationBounds(INTERGRAL_MIN, INTERGRAL_MAX);
    }

    public void tankController(double right, double left) {
        leftMaster.setPower(left);
        leftFollow.setPower(left);

        rightMaster.setPower(right);
        rightFollow.setPower(right);
    }

    public void arcadeController(double forward, double rotation) {
        double left = forward + rotation;
        double right = forward - rotation;
        double max = Math.max(left, right);
        if(max > 1) {
            left /= max;
            right /= max;
        }
        leftMaster.setPower(left);
        leftFollow.setPower(left);
        rightMaster.setPower(right);
        rightFollow.setPower(right);
    }

    public int getLeftPosition() {
        return leftMaster.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightMaster.getCurrentPosition();
    }

    public double rotateAngle(double angle, double setPoint) {
        controller.setSetPoint(setPoint);
        return controller.calculate(angle);
    }

    public boolean atSetpoint() {
        return controller.atSetPoint();
    }
}
