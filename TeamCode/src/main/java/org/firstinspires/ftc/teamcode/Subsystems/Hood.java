package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Hood {
    private Servo hood;
    private HardwareMap hardwareMap;
    public Hood(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");

        hood.scaleRange(0, 1);
        hood.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        hood.setPosition(position);
    }

    public double getPosition() {
        return hood.getPosition();
    }

    public boolean hasReached(double position) {
        return Math.abs(getPosition()-position) == Constants.SHOOT.HOOD_TOLERANCE;
    }
}
