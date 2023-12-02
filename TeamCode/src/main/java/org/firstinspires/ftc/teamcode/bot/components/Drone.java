package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    Servo droneServo;

    private boolean droneMode = false;

    public Drone (HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class, "drone");
        droneServo.setPosition(0.5);
    }

    public void changeDroneMode (boolean pressed) {
        if (pressed) {
            droneMode = !droneMode;
        }
    }

    public void launchDrone (boolean pressed) {
        if (droneMode) {
            if (pressed) {
                droneServo.setPosition(0.8);
            } else {
                droneServo.setPosition(0.5);
            }
        }
    }

    public String getDroneMode () {
        if (droneMode) {
            return "Drone Mode On";
        } else {
            return "Drone Mode Off";
        }
    }
}



