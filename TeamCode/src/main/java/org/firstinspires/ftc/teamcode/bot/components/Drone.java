package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    Servo droneServo;
    private int launchlvl = 60;
    private double[] dronePos= {0.5, 0.8};

    /**
     * makes drone object in robot, start position is .5
     * @param hardwareMap is how the code interact with the robot
     */
    public Drone (HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class, "drone");
        droneServo.setPosition(0.5);
    }

    /**
     * cycles through positions, updating position
     * @param increase is based on whether "a" button is pressed
     */
    public void launchDrone (boolean increase) {
        if (increase) {
            launchlvl += 1;
        }
        setLaunchPosition();
    }

    /**
     * sets position, updates based on launchDrone
     */
    public void setLaunchPosition() {
        droneServo.setPosition(dronePos[Math.abs(launchlvl % dronePos.length)]);
    }
}



