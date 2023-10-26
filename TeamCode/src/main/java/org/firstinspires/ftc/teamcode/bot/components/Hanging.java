package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanging {
    private CRServo hangServo1;
    private CRServo hangServo2;

    public Hanging (HardwareMap hardwareMap){
        hangServo1 = hardwareMap.get(CRServo.class, "hang1");
        hangServo2 = hardwareMap.get(CRServo.class, "hang2");
    }

    public void hang(boolean pressing) {
        if (pressing) {
            hangServo1.setPower(1);
            hangServo2.setPower(-1);
        } else {
            hangServo1.setPower(0);
            hangServo2.setPower(0);
        }
    }
}


