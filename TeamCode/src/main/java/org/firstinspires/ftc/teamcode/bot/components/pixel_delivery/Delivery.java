package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Delivery {
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    private CRServo extensionServo;
    private Servo droppingServo;

    private int[] liftMotorPos = {0, 200, 400, 600, 1000};
    private double[] dropServoPos = {0, 0.5, 0.6};
//    private double[] extendServoPos = {0, 0.2, 0.4, 0.6, 1};

    private int liftLvl = 60;
    private int dropLvl = 60;
//    private int extendLvl = 60;

    public Delivery (HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift2");
        extensionServo = hardwareMap.get(CRServo.class, "extend");
        droppingServo = hardwareMap.get(Servo.class, "drop");

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionServo.setDirection(DcMotorSimple.Direction.REVERSE);
        droppingServo.setPosition(0);
    }

    public void driveLift (double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public void extendOuttake (double power) {
        extensionServo.setPower(power);
    }

    public void changeLiftHeight (boolean increase) {
        if (increase) {
            liftLvl += 1;
        }
        setDeliveryPositions();
    }

    public void changeDropPosition (boolean increase) {
        if (increase) {
            dropLvl += 1;
        }
        setDeliveryPositions();
    }

 //    public void changeExtendHeight (boolean increase) {
//        if (increase) {
//            extendLvl += 1;
//        }
//        setDeliveryHeights();
//    }

    private void setDeliveryPositions() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor1.setTargetPosition(liftMotorPos[Math.abs(liftLvl % liftMotorPos.length)]);
        droppingServo.setPosition(dropServoPos[Math.abs(dropLvl % dropServoPos.length)]);
//        extensionServo.setPosition(extendServoPos[Math.abs(extendLvl % extendServoPos.length)]);
    }

    public int getLiftPosition() {
        return liftMotor1.getCurrentPosition();
    }

    public double getDropPosition () {
        return droppingServo.getPosition();
    }

//    public double getExtendPosition () {
//        return extensionServo.getPosition();
//    }

    private int getLiftLvl() {
        return Math.abs(liftLvl % liftMotorPos.length);
    }

    private int getDropLvl() {
        return Math.abs(dropLvl % dropServoPos.length);
    }

//    private int getExtendLvl() {
//        return Math.abs(extendLvl % extendServoPos.length);
//    }

    public void resetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
