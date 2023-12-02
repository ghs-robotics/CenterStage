package org.firstinspires.ftc.teamcode.bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.Drone;
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
    public Drone drone;

    public boolean RED;

    FtcDashboard dashboard;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean red){
        this(hardwareMap, telemetry);
        this.RED = red;

    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
//        dashboard = FtcDashboard.getInstance();
//        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        this.telemetry = telemetry;
        gyro = new Gyro(hardwareMap);
        drive = new BallDrive(hardwareMap, gyro);

        nav = new Navigation(drive, gyro, telemetry);
        intake = new Intake(hardwareMap);
        delivery = new Delivery(hardwareMap);
        drone = new Drone(hardwareMap);

        cam = new Camera(hardwareMap, telemetry, RED);

    }

    /**
     * initializes the robot parts
     */
    public void init(){
        //init cameras
        cam.initCamera();
        gyro.resetHeading();
        drive.resetEncoders();
        delivery.resetEncoders();
        nav.resetNav();
    }

    public void shutOff(){
        drive.calculateDrivePowers(0,0,0);
        intake.pixelIn(0);
        delivery.driveLift(0);
    }

    /**
     * tells the robot parts to retrieve the current information from each part to update the robot.
     */
    public void update(){
        nav.update();
        telemetry.update();
    }

    public void getAutoTelemetry(){
        positionTelemetry();
    }

    public void getTeleOpTelemetry(){
        positionTelemetry();
        intakeTelemetry();
        deliveryTelemetry();
        droneTelemetry();
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
        telemetry.addData("intake position: ", intake.getIntakePosition());
        telemetry.addLine();
    }

    private void deliveryTelemetry () {
        telemetry.addLine("Delivery System Telemetry");
        telemetry.addData("lift position: ", delivery.getLiftPosition());
//        telemetry.addData("extension position: ", delivery.getExtensionPosition());
        telemetry.addData("drop position", delivery.getDropPosition());
        telemetry.addData("touch sensor status", delivery.getTouchSensorStatus());
        telemetry.addLine();
    }

    private void droneTelemetry () {
        telemetry.addLine("Drone System Telemetry");
        telemetry.addData("Drone Mode Status: ", drone.getDroneMode());
        telemetry.addLine();
    }
}
