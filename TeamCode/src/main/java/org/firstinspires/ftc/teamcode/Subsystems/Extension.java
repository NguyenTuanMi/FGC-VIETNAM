package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extension {
    private CRServo motor1;
    private CRServo motor2;
    private HardwareMap hardwareMap;

    public Extension(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        motor1 = hardwareMap.get(CRServo.class, "mr1");
        motor2 = hardwareMap.get(CRServo.class, "mr2");

        motor1.resetDeviceConfigurationForOpMode();
        motor2.resetDeviceConfigurationForOpMode();

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void extend(double speed) {
        motor1.setPower(speed);
        motor2.setPower(speed);
    }
}
