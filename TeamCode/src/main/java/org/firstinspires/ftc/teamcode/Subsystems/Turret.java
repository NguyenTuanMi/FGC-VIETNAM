package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotorEx turret;
    private HardwareMap hardwareMap;

    public Turret(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "tr");

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void tuning(double speed) {
        turret.setVelocity(speed);
    }

    public double getVelocity(){
        return turret.getVelocity();
    }
}
