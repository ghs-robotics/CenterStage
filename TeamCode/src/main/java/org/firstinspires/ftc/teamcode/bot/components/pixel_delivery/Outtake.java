package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    private CRServo extensionServo;
    private Servo droppingServo;

    private ElapsedTime timer;

    public Outtake (HardwareMap hardwareMap) {
        extensionServo = hardwareMap.get(CRServo.class, "extend");
        droppingServo = hardwareMap.get(Servo.class, "drop");
        timer.reset();
    }

    public void pixelOut () {
        extensionServo.setPower(1);
    }
}
