package org.firstinspires.ftc.teamcode.robot_components;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    BallDrive drive;
    Gyro gyro;

    int x;
    int y;
    int heading;

    public Robot(){
        drive = new BallDrive(hardwareMap, telemetry);
        gyro = new Gyro(hardwareMap);
    }
}
