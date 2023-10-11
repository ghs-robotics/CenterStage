package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    public static final int MIN = 0;
    public static final int MAX = 100;
    public static final int LOW = 20;
    public static final int MID = 40;
    public static final int HIGH = 60;

//    double[] pos = {};
//
//    private int liftLvl;

    public Lift (HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void moveToLow () {
        liftMotor1.setTargetPosition(LOW);
    }

    public void moveToMid () {
        liftMotor1.setTargetPosition(MID);
    }

    public void moveToHigh () {
        liftMotor1.setTargetPosition(HIGH);
    }

//    public void changeLiftHeight(boolean decrease, boolean increase) {
//        if (decrease) {
//            liftLvl -= 1;
//        }
//        if (increase) {
//            liftLvl += 1;
//        }
//    }

    public int getLiftPosition(){
        return liftMotor1.getCurrentPosition();
    }

//    private void setHeight () {
//        liftMotor1.setPower(pos[Math.abs(liftLvl % pos.length)]);
//    }

    public void resetEncoders(){
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
