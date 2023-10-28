package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Delivery {
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    private Servo extensionServo;
    private Servo droppingServo;

    private double[] liftMotorPos = {0, 20, 40, 60, 100};
    private double[] dropServoPos = {0, 0.2};
    private double[] extendServoPos = {0, 0.2, 0.4, 0.6, 1};

    private int liftLvl = 60;
    private int dropLvl = 60;
    private int extendLvl = 60;

    public Delivery(HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift2");
        extensionServo = hardwareMap.get(Servo.class, "extend");
        droppingServo = hardwareMap.get(Servo.class, "drop");
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void raiseLift () {
        if (liftMotor1.getCurrentPosition() >= 0 && liftMotor1.getCurrentPosition() <= 100) {
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        }
    }

    public void driveLift(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public void changeLiftHeight (boolean increase) {
        if (increase) {
            liftLvl += 1;
        }
        setDeliveryHeights();
    }

    public void changeDropHeight (boolean increase) {
        if (increase) {
            dropLvl += 1;
        }
        setDeliveryHeights();
    }

    public void changeExtendHeight (boolean increase) {
        if (increase) {
            extendLvl += 1;
        }
        setDeliveryHeights();
    }

    private void setDeliveryHeights () {
        liftMotor1.setTargetPosition((int) liftMotorPos[Math.abs(liftLvl % liftMotorPos.length)]);
        droppingServo.setPosition(dropServoPos[Math.abs(dropLvl % dropServoPos.length)]);
        extensionServo.setPosition(extendServoPos[Math.abs(extendLvl % extendServoPos.length)]);
    }

    public int getLiftPosition() {
        return liftMotor1.getCurrentPosition();
    }

    private int getLiftLvl() {
        return Math.abs(liftLvl % liftMotorPos.length);
    }

    private int getDropLvl() {
        return Math.abs(dropLvl % dropServoPos.length);
    }

    private int getExtendLvl() {
        return Math.abs(extendLvl % extendServoPos.length);
    }

    public int getLiftPos () {
        return getLiftLvl();
    }

    public int getDropPos () {
        return getDropLvl();
    }

    public int getExtendPos () {
        return getExtendLvl();
    }

    public void resetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
