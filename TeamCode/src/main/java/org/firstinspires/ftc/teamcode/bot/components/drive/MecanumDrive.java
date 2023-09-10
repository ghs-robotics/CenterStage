package org.firstinspires.ftc.teamcode.bot.components.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive implements Drivebase{

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;


    public MecanumDrive(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotor.class,"FLDrive");
        backLeft = hardwareMap.get(DcMotor.class,"BLDrive");
        frontRight = hardwareMap.get(DcMotor.class,"FRDrive");
        backRight = hardwareMap.get(DcMotor.class,"BRDrive");
    }

    @Override
    public void calculateDrivePowers(double x, double y, double rot) {
        rot = -rot;
        double frontLeftPower = rot - x + y;
        double backLeftPower = rot + x + y;
        double frontRightPower = rot - x - y;
        double backRightPower = rot + x - y;

        setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    @Override
    public int[] getEncoderTicks() {
        int[] ticks = {frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition()};
        return ticks;
    }

    private void setMotorPowers(double fl, double bl, double fr, double br){
            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            frontRight.setPower(fr);
            backRight.setPower(br);
    }
}
