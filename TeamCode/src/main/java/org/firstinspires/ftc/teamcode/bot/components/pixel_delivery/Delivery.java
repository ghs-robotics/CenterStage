package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Delivery {
    private final DcMotor liftMotor1;
    private final DcMotor liftMotor2;

    private final Servo extendServo;
    private final Servo dropServo;

    private final TouchSensor touchSensor1;

    private final TouchSensor touchSensor2;

    private final double[] extendServoPos = {0, 0.1, 0.2, 0.3, 0.4};

    private int extendLvl = 90;

    public static final double DROPPER_INTAKING = 0.1;
    public static final double DROPPER_FIRST = 0.5;
    public static final double DROPPER_SECOND = 0.6;

    private double sentPower;

    private boolean tuneLiftMode = false;

    public Delivery (HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift2");
        extendServo = hardwareMap.get(Servo.class, "extend");
        dropServo = hardwareMap.get(Servo.class, "drop");

        touchSensor1 = hardwareMap.get(TouchSensor.class, "touch1");
        touchSensor2 = hardwareMap.get(TouchSensor.class, "touch2");

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE); // currently polarity is reversed
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        extendServo.setDirection(DcMotorSimple.Direction.REVERSE);

        dropServo.setPosition(0.15);
        extendServo.setPosition(0);
    }

    //-------------------------------------------------------------------------------------
    //                                   AutoRed Functions
    //-------------------------------------------------------------------------------------

//    public boolean autoRunExtension(double dir, double curMillisecond){
//        if (curMillisecond < 350){
//            setExtendPower(dir);
//        } else
//            setExtendPower(0);
//        return curMillisecond > 500;
//    }

    public boolean autoDropPixels(double targetPos){
        dropServo.setPosition(targetPos);
        return dropServo.getPosition() == targetPos;
    }

    public boolean driveLiftToPosition(int target){
        if (getLiftPosition() < target - 5 || getLiftPosition() > target + 5)
            driveLift((getLiftPosition() - target) / 10.0);
        else
            driveLift(-0.1);
        return getLiftPosition() < target - 5 || getLiftPosition() > target + 5;
    }

    //-------------------------------------------------------------------------------------
    //                                   Lift Functions
    //-------------------------------------------------------------------------------------

    public void driveLift (double power) {
        touchSensorEncoderReset();
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sentPower = power;

        if (!tuneLiftMode) {
            if (Math.abs(power) > 0.1) {
                limitLift(power);
            } else if (Math.abs(power) < 0.1) {
                setLiftPower(0);
            }
        }
    }

    public void changeLiftMode (boolean pressed) {
        if (pressed) {
            tuneLiftMode = !tuneLiftMode;
        }
    }

    public void tuneLiftDuringTele (double liftMotorPower1, double liftMotorPower2) {
        if (tuneLiftMode) {
            this.liftMotor1.setPower(liftMotorPower1);
            liftMotor2.setPower(liftMotorPower2);
        }
    }

    public void touchSensorEncoderReset () {
        if (getTouchSensor1Status()) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (getTouchSensor2Status()) {
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private void limitLift(double power){
        int limit = 1500;

        if (getLiftPosition() > limit && power > 0) {
            power = 0;
        } else if (getLiftPosition() > limit - 150){
            power *= (limit - getLiftPosition()) / 200.0;
        } else if (getLiftPosition() <= 0 && power > 0){
            power = 0;
        }
        setLiftPower(power);
    }

    //-------------------------------------------------------------------------------------
    //                                   Extension Functions
    //-------------------------------------------------------------------------------------

    public void changeExtensionLength (boolean decrease, boolean increase) {
        if (decrease) {
            extendLvl -= 1;
        }
        if (increase) {
            extendLvl += 1;
        }
        setExtensionPosition();
    }

    //-------------------------------------------------------------------------------------
    //                                   Drop Functions
    //-------------------------------------------------------------------------------------

    public void changeDropPosition(boolean pressing) {
        if (!tuneLiftMode) {
            if (pressing) {
                dropServo.setPosition(0.4);
            } else {
                dropServo.setPosition(0.15);
            }
        }
    }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    public void resetEncoders () {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setLiftPower(double power){
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public boolean getTuneLiftMode () {
        return tuneLiftMode;
    }

    public int getLiftPosition () {
        return -liftMotor1.getCurrentPosition();
    }

    private void setExtensionPosition () {
        extendServo.setPosition(extendServoPos[Math.abs(extendLvl % extendServoPos.length)]);
    }

    public double getExtensionPosition () {
        return extendServo.getPosition();
    }

    public double getDropPosition () {
        return dropServo.getPosition();
    }

    public boolean getTouchSensor1Status () {
        return touchSensor1.isPressed();
    }
    public boolean getTouchSensor2Status () {
        return touchSensor2.isPressed();
    }
}
