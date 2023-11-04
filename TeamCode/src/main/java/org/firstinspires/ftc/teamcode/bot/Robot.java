package org.firstinspires.ftc.teamcode.bot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.bot.components.Drone;
import org.firstinspires.ftc.teamcode.bot.components.Hanging;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.bot.components.drive.BallDrive;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;
import org.firstinspires.ftc.teamcode.bot.control.Navigation;
import org.firstinspires.ftc.teamcode.cv.Camera;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Camera cam;

    public Drivebase drive;
    public Navigation nav;
    private Gyro gyro;

    public Intake intake;
    public Delivery delivery;
    public Hanging hang;
//    public Drone drone;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        gyro = new Gyro(hardwareMap);
        drive = new BallDrive(hardwareMap, gyro);

        nav = new Navigation(drive, gyro, telemetry);
        intake = new Intake(hardwareMap);
        delivery = new Delivery(hardwareMap);
        hang = new Hanging(hardwareMap);

        cam = new Camera();

    }

    /**
     * initializes the robot parts
     */
    public void init(){
        //init cameras
        gyro.resetHeading();
        drive.resetEncoders();
        delivery.resetEncoders();
    }

    /**
     * tells the robot parts to retrieve the current information from each part to update the robot.
     */
    public void update(){
        nav.updatePosition();
        //delivery.preventDropperDamage();
        telemetry.update();
    }

    public void getTelemetry(){
        positionTelemetry();
        intakeTelemetry();
        deliveryTelemetry();
    }

    private void positionTelemetry(){
        telemetry.addLine("Drivebase Telemetry");
        telemetry.addData("Meta Drive Mode On: ", drive.getDriveMode());
        telemetry.addData("x pos: ", nav.getX());
        telemetry.addData("y pos: ", nav.getY());
        telemetry.addData("gyro heading: ", Math.toDegrees(nav.getGyroHeading()));
        telemetry.addLine();
    }

    private void intakeTelemetry(){
        telemetry.addLine("Intake Telemetry");
        telemetry.addData("intake position: ", intake.getIntakePos());
        telemetry.addLine();
    }

    private void deliveryTelemetry () {
        telemetry.addLine("Delivery System Telemetry");
        telemetry.addData("lift position: ", delivery.getLiftPosition());
        telemetry.addData("lift target Pos: ", delivery.getLiftLvl());
        telemetry.addData("Lift Preset On: ", delivery.getLiftMode());
        telemetry.addData("drop position", delivery.getDropPosition());
//        telemetry.addData("extend position", deliver.getExtendPosition());
        telemetry.addLine();
    }
}
