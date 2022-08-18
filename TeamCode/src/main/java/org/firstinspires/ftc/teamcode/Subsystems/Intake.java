package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServo mr1;
    private CRServo mr2;
    private HardwareMap hardwareMap;

    public Intake(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        mr1 = hardwareMap.get(CRServo.class, "intake1");
        mr2 = hardwareMap.get(CRServo.class, "intake2");

        mr1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setSpeed(double speed) {
        mr1.setPower(speed);
        mr2.setPower(speed);
    }
 }
