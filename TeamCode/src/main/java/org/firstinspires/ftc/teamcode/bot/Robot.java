package org.firstinspires.ftc.teamcode.bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.Drone;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;
import org.firstinspires.ftc.teamcode.control.cv.Camera;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Camera cam;

    public BallDrive drive;
    private final Gyro gyro;

    public Intake intake;
    public Delivery delivery;
    public Drone drone;

    FtcDashboard dashboard;

    public boolean RED;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean red){
        this(hardwareMap, telemetry);
        cam = new Camera(hardwareMap, telemetry, red);
        RED = red;
        cam.setCamera();
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
//        dashboard = FtcDashboard.getInstance();
//        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        this.telemetry = telemetry;
        gyro = new Gyro(hardwareMap);
//        drive = new BallDrive(hardwareMap, gyro);

        intake = new Intake(hardwareMap);
        delivery = new Delivery(hardwareMap);
        drone = new Drone(hardwareMap);

    }

    /**
     * initializes the robot parts
     */
    public void init(){
        //init cameras
        gyro.resetHeading();
//        drive.resetCoords();
        delivery.resetEncoders();
    }

    public void shutOff(){
//        drive.calculateDrivePowers(0,0,0);
        intake.pixelIn(0);
        delivery.driveLift(0);
        cam.closeCamera();
    }

    /**
     * tells the robot parts to retrieve the current information from each part to update the robot.
     */
    public void update(){
//        drive.update();
        telemetry.update();
    }

    public void getAutoTelemetry(){
        cam.getTelemetry();
        positionTelemetry();
    }

    public void getTeleOpTelemetry(){
        positionTelemetry();
        intakeTelemetry();
        deliveryTelemetry();
    }

    private void positionTelemetry(){
//        telemetry.addLine("Drivebase Telemetry");
//        telemetry.addData("Meta Drive Mode On: ", drive.getDriveMode());
//        telemetry.addData("x pos: ", drive.getX());
//        telemetry.addData("y pos: ", drive.getY());
//        telemetry.addLine();
//        telemetry.addData("x out", drive.getXError());
//        telemetry.addData("gyro heading: ", drive.getHeading());
//        telemetry.addLine(String.valueOf(drive.resetCounter));
//        telemetry.addLine();
    }

    private void intakeTelemetry(){
        telemetry.addLine("Intake Telemetry");
        telemetry.addData("intake position: ", intake.getIntakePosition());
        telemetry.addLine();
    }

    private void deliveryTelemetry () {
        telemetry.addLine("Delivery System Telemetry");
        telemetry.addData("hang mode status:", delivery.getHangModeStatus());
        telemetry.addData("lift position: ", delivery.getLift1Position());
        telemetry.addData("lift position: ", delivery.getLift2Position());
        telemetry.addData("extension position: ", delivery.getExtensionPosition());
        telemetry.addData("drop position", delivery.getDropPosition());
//        telemetry.addData("touch sensor status", delivery.getTouchSensorStatus());
        telemetry.addLine();
    }
}
