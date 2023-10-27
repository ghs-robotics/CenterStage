package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drone {
    DcMotor droneMotor;

    public Drone (HardwareMap hardwareMap) {
        droneMotor = hardwareMap.get(DcMotor.class, "droneMotor");
    }

    public void deliverDrone (boolean pressing) {
        if (pressing) {
            droneMotor.setPower(1);
        } else {
            droneMotor.setPower(0);
        }
    }
}
