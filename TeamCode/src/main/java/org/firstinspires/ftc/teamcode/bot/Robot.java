package org.firstinspires.ftc.teamcode.bot;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.bot.components.drive.BallDrive;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Lift;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Outtake;
import org.firstinspires.ftc.teamcode.bot.control.Navigation;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Drivebase drive;
    public Navigation nav;
    private Gyro gyro;

    public Intake intake;
    public Lift lift;
    public Outtake outtake;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        drive = new BallDrive(hardwareMap);
        gyro = new Gyro(hardwareMap);

        nav = new Navigation(drive, gyro);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

    public void init(){
        //init cameras
        drive.resetEncoders();
        lift.resetEncoders();
    }

    public void update(){
        nav.updatePosition();
    }

    public void getTelemetry(){
        liftTelemetry();
        intakeTelemetry();
        positionTelemetry();
    }

    private void positionTelemetry(){
        telemetry.addData("x pos: ", nav.getX());
        telemetry.addData("y pos: ", nav.getY());
        telemetry.addData("odo heading: ", Math.toDegrees(nav.getOdoHeading()));
        telemetry.addData("gyro heading: ", Math.toDegrees(nav.getGyroHeading()));
        telemetry.addLine();
    }

    private void intakeTelemetry(){
        telemetry.addData("intake position: ", intake.getIntakePos());
        telemetry.addLine();
    }

    private void liftTelemetry(){
        telemetry.addData("lift position: ", lift.getLiftPosition());
        telemetry.addLine();
    }
}
