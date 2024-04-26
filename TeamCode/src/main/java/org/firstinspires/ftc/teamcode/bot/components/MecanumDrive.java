package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    public MecanumDrive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class,"FRDrive");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRDrive");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void calculateDrivePower(double x, double y, double rot){
        rot = -rot;
        double frontLeft = rot - x + y;
        double backLeft = rot + x + y;
        double frontRight = rot - x - y;
        double backRight = rot + x - y;

        sendDrivePower(frontLeft, backLeft, frontRight, backRight);
    }

    public void sendDrivePower(double frontLeft, double backLeft, double frontRight, double backRight){
        //reversed to match motor polarity or something
        frontLeftDrive.setPower(frontLeft);
        backLeftDrive.setPower(backLeft);
        frontRightDrive.setPower(frontRight);
        backRightDrive.setPower(backRight);
    }
}
