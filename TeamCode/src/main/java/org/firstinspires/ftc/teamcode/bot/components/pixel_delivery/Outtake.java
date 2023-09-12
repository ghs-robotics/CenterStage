package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    CRServo outServo;

    public Outtake (HardwareMap hardwareMap, Telemetry telemetry) {
        outServo = hardwareMap.get(CRServo.class, "outServo");
        telemetry.update();
    }

    public void pixelOut () {
        outServo.setPower(1);
    }
}
