package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    public static final int MIN = 0;
    public static final int MAX = 500;
    public static final int LOW = 100;
    public static final int MID = 200;
    public static final int HIGH = 500;

    public Lift (HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
    }

    public void raiseLift () {
        if (liftMotor1.getCurrentPosition() >= MIN && liftMotor1.getCurrentPosition() <= MAX) {
            liftMotor1.setPower(1);
        }
    }

    public void driveLift(double power){
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setLow () {
        liftMotor1.setTargetPosition(LOW);
    }

    public void setMid () {
        liftMotor1.setTargetPosition(MID);
    }

    public void setHigh () {
        liftMotor1.setTargetPosition(HIGH);
    }

    public int getLiftPosition(){
        return liftMotor1.getCurrentPosition();
    }

    public void resetEncoders(){
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
