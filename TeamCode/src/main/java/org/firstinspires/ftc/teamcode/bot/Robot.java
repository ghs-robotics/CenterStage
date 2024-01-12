package org.firstinspires.ftc.teamcode.bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.Drone;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.bot.components.drive.BallDrive;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;
import org.firstinspires.ftc.teamcode.bot.control.Navigation;
import org.firstinspires.ftc.teamcode.cv.Camera;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Camera cam;

    public BallDrive drive;
    public Navigation nav;
    private Gyro gyro;

    public Intake intake;
    public Delivery delivery;
    public Drone drone;

    FtcDashboard dashboard;

    public boolean RED;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean red){
        this(hardwareMap, telemetry);
        cam = new Camera(hardwareMap, telemetry, red);
        RED = red;
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

//        dashboard.startCameraStream(cam.camera1, 0);
    }

    /**
     * initializes the robot parts
     */
    public void init(){
        //init cameras
        nav.resetNav();
        gyro.resetHeading();
        drive.resetEncoders();
        drive.resetCoords();
        delivery.resetEncoders();
    }

    public void shutOff(){
        drive.calculateDrivePowers(0,0,0);
        drive.resetCoords();
        intake.pixelIn(0);
        delivery.driveLift(0, 0);
        nav.resetNav();
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
    }

    private void positionTelemetry(){
        telemetry.addLine("Drivebase Telemetry");
        telemetry.addData("Meta Drive Mode On: ", drive.getDriveMode());
        telemetry.addData("x pos: ", nav.getX());
        telemetry.addData("y pos: ", nav.getY());
        telemetry.addData("gyro heading: ", Math.toDegrees(nav.getHeading()));
        telemetry.addLine();
    }

    private void intakeTelemetry(){
        telemetry.addLine("Intake Telemetry");
        telemetry.addData("intake position: ", intake.getIntakePosition());
        telemetry.addLine();
    }

    private void deliveryTelemetry () {
        telemetry.addLine("Delivery System Telemetry");
        telemetry.addData("lift back to zero status: ", delivery.getLiftBackToZeroStatus());
        telemetry.addData("lift position 1: ", delivery.getLift1Position());
        telemetry.addData("lift position 2: ", delivery.getLift2Position());
        telemetry.addData("extension position: ", delivery.getExtensionPosition());
        telemetry.addData("drop position", delivery.getDropPosition());
        telemetry.addData("touch sensor 1 status", delivery.getTouchSensor1Status());
        telemetry.addData("touch sensor 2 status", delivery.getTouchSensor2Status());
        telemetry.addLine();
    }
}
