package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;

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

    public void launchDrone (boolean pressing) {
            if (droneMode) {
                if (pressing) {
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



