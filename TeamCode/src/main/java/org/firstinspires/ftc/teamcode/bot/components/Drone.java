package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drone {
    private CRServo droneServo;

    public Drone (HardwareMap hardwareMap){
        droneServo = hardwareMap.get(CRServo.class, "drone");
    }

    public void launch(boolean pressing) {
        if (pressing) {
            droneServo.setPower(1);
        } else {
            droneServo.setPower(0);
        }
    }
}
