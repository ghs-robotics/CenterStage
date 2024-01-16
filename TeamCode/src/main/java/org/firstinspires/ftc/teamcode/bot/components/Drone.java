package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;

public class Drone {
    private final Servo droneServo;

    public Drone (HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class, "drone");
        droneServo.setPosition(0.5);
    }

    public void launchDrone (boolean pressing) {
                if (pressing) {
                    droneServo.setPosition(0.8);
                } else {
                    droneServo.setPosition(0.5);
                }
            }
}



