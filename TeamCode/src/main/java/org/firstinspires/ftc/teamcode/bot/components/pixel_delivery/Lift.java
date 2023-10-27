package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    private Servo extensionServo;
    private Servo droppingServo;

    public static final int MIN = 0;
    public static final int MAX = 100;
    public static final int LOW = 20;
    public static final int MID = 40;
    public static final int HIGH = 60;

    public static int EXTLVLLOW = 0;
    public static int EXTLVLMID = 1;

    public static final double EXTMIN = 0;
    public static final double EXTMAX = 1.0;
    public static final double EXTLOW = .2;
    public static final double EXTMID = .4;
    public static final double EXTHIGH = .6;

    public static final double INTAKE = 0;
    public static final double DR1 = 1.0;
    public static final double EMPTY = .5;



    public Lift(HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift2");
        extensionServo = hardwareMap.get(Servo.class, "extend");
        droppingServo = hardwareMap.get(Servo.class, "drop");
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void raiseLift () {
        if (liftMotor1.getCurrentPosition() >= MIN && liftMotor1.getCurrentPosition() <= MAX) {
            liftMotor1.setPower(1);
        }
    }

    public void driveLift(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public void moveToLow() {
        liftMotor1.setTargetPosition(LOW);
    }

    public void moveToMid() {
        liftMotor1.setTargetPosition(MID);
    }

    public void moveToHigh() {
        liftMotor1.setTargetPosition(HIGH);
    }

    public int getLiftPosition() {
        return liftMotor1.getCurrentPosition();
    }

    public void resetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveToDropperLow() {
        droppingServo.setPosition(INTAKE);
    }
    public void moveToDropperMid() {
        droppingServo.setPosition(EMPTY);
    }
    public void moveToDropperHigh() {
        droppingServo.setPosition(DR1);
    }
    public void moveToExtenderLow() {
        extensionServo.setPosition(EXTLOW);

    }
    public void moveToExtenderMid() {
        extensionServo.setPosition(EXTMID);

    }
    public void moveToExtenderHigh() {
        extensionServo.setPosition(EXTHIGH);

    }

}
