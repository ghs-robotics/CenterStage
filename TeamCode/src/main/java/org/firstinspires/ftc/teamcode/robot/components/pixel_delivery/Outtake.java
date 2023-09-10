package org.firstinspires.ftc.teamcode.robot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    CRServo outtakeServo1;
    CRServo outtakeServo2;

    public Outtake (HardwareMap hardwareMap, Telemetry telemetry) {
        outtakeServo1 = hardwareMap.get(CRServo.class, "outMotor1");
        outtakeServo2 = hardwareMap.get(CRServo.class, "outMotor2");
    }
    public void delivery (double deli) {
        outtakeServo1.setPower(deli);
        outtakeServo2.setPower(deli);
    }
}
