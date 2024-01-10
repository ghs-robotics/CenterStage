package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.bot.control.PID;
import org.firstinspires.ftc.teamcode.bot.control.TestPID;

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

    private final TestPID pid;

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

        dropServo.setPosition(0.15);
        extendServo.setPosition(0);

        pid = new TestPID();
    }

    //-------------------------------------------------------------------------------------
    //                                   AutoRed Functions
    //-------------------------------------------------------------------------------------

    public boolean autoDropPixels(double targetPos){
        dropServo.setPosition(targetPos);
        return dropServo.getPosition() == targetPos;
    }

//    public boolean driveLiftToPosition(int target){
//        if (getLift1Position() < target - 5 || getLift1Position() > target + 5)
//            driveLift((getLift1Position() - target) / 10.0);
//        else
//            driveLift(-0.1);
//        return getLift1Position() < target - 5 || getLift1Position() > target + 5;
//    }

    //-------------------------------------------------------------------------------------
    //                                   Lift Functions
    //-------------------------------------------------------------------------------------

    public void driveLift(double power1, double power2) {
        touchSensorEncoderReset();
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (getLift1Position() <= 0 && power1 > 0) {
            liftMotor1.setPower(0);
        } else {
            liftMotor1.setPower(power1);
        }

        if (getLift2Position() <= 0 && power2 > 0) {
            liftMotor2.setPower(0);
        } else {
            liftMotor2.setPower(power2);
        }
    }

    public void touchSensorEncoderReset () {
        if (getTouchSensor1Status()) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (getTouchSensor2Status()) {
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
            if (pressing) {
                dropServo.setPosition(0.4);
            } else {
                dropServo.setPosition(0.15);
            }
        }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    public void resetEncoders () {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getLift1Position() {
        return -liftMotor1.getCurrentPosition();
    }

    public double getLift2Position () {
        return liftMotor2.getCurrentPosition();
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
