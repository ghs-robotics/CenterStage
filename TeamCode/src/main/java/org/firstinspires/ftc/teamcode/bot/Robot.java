package org.firstinspires.ftc.teamcode.bot;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.bot.components.drive.BallDrive;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;
import org.firstinspires.ftc.teamcode.bot.control.Navigation;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Drivebase drive;
    private Gyro gyro;

    Navigation nav;

    Intake intake;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        drive = new BallDrive(hardwareMap);
        gyro = new Gyro(hardwareMap);

        nav = new Navigation(drive, gyro);
    }

    public void init(){
        //init cameras
        drive.resetEncoders();
    }

    public void update(){
        nav.updatePosition();
    }

    public void getTelemetry(){
        positionTelemetry();
    }

    private void positionTelemetry(){
        telemetry.addData("x pos: ", nav.getX());
        telemetry.addData("y pos: ", nav.getY());
        telemetry.addData("odo heading: ", Math.toDegrees(nav.getOdoHeading()));
        telemetry.addData("gyro heading: ", Math.toDegrees(nav.getGyroHeading()));
        telemetry.addLine();
    }
}
