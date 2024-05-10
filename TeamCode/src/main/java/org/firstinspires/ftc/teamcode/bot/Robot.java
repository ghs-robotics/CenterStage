package org.firstinspires.ftc.teamcode.bot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.Drone;
import org.firstinspires.ftc.teamcode.bot.components.Led;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.DistanceSensor;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.bot.components.BallDrive;
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

    public DistanceSensor distance;
    public Drone drone;

    public Led led;

    public boolean RED;

    /**
     * Overloaded Robot constructor built for Auto because it will initialize teh camera.
     *
     * @param hardwareMap Access to the rev hub
     * @param telemetry Access to display info on the driver hub
     * @param red Which side of the field we start on
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean red){
        this(hardwareMap, telemetry);
        cam = new Camera(hardwareMap, telemetry, red);
        RED = red;
        cam.setCamera();
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;

        this.telemetry = telemetry;
        gyro = new Gyro(hardwareMap);
//        drive = new BallDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive = new BallDrive(hardwareMap, gyro);

        intake = new Intake(hardwareMap);
        delivery = new Delivery(hardwareMap);
        distance = new DistanceSensor(hardwareMap);
        drone = new Drone(hardwareMap);

        led = new Led(hardwareMap);
    }

    /**
     * initializes the robot parts
     */
    public void init(){
        //init cameras
        gyro.resetHeading();
        drive.resetCoords();
        delivery.resetEncoders();
        led.ledsOff();
        distance.addInitialDataToDistanceArray();
    }

    /**
     * Sets all motors and devices to 0 power, effectively stopping all robot movement.
     */
    public void shutOff(){
        drive.calculateDrivePowers(0,0,0);
        intake.pixelIn(0);
        delivery.driveLift(0);
//        cam.closeCamera();
        led.ledsOff();
    }

    /**
     * tells the robot parts to retrieve the current information from each part to update the robot.
     */
    public void update(){
        drive.update();
        telemetry.update();
        distance.countPixels(intake);
        led.setNumPixels(distance.getPixelNumber());
        led.runLeds();
    }

    public void getAutoTelemetry(){
        cam.getTelemetry();
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
        telemetry.addData("x pos: ", drive.getX());
        telemetry.addData("y pos: ", drive.getY());
        telemetry.addLine();
//        telemetry.addData("x out", drive.getXError());
        telemetry.addData("gyro heading: ", drive.getHeading());
//        telemetry.addLine(String.valueOf(drive.resetCounter));
//        telemetry.addLine();
    }

    private void intakeTelemetry(){
        telemetry.addLine("Intake Telemetry");
        telemetry.addData("intake position: ", intake.getIntakePosition());
        telemetry.addData("number of pixels:", distance.getPixelNumber());
        telemetry.addData("distance from pixel:", distance.getDistanceFromPixel());
        telemetry.addData("pixel distances:", distance.getPixelDistances());
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

    private void droneTelemetry () {
        telemetry.addData("drone position:", drone.getDronePosition());
    }
}
