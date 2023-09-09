package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.robot.components.drive.BallDrive;
import org.firstinspires.ftc.teamcode.robot.components.Gyro;
import org.firstinspires.ftc.teamcode.robot.components.drive.Drivebase;
import org.firstinspires.ftc.teamcode.robot.control.Navigation;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Drivebase drive;
    Gyro gyro;

    Navigation nav;

    Intake intake;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        drive = new BallDrive(hardwareMap);
        gyro = new Gyro(hardwareMap);


    }
}
