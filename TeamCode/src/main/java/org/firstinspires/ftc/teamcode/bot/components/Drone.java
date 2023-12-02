package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    Servo droneServo;
    private int launchlvl = 60;
    private double[] droneServoPos = {0.5, 0.8};

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

    public void launchDrone (boolean increase) {
        if (droneMode) {
            if (increase) {
                launchlvl += 1;
            }
            setLaunchPosition();
        }
    }

    public void setLaunchPosition() {
        droneServo.setPosition(droneServoPos[Math.abs(launchlvl % droneServoPos.length)]);
    }
}



