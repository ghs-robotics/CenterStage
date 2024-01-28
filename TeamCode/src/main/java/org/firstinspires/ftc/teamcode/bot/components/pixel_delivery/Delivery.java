package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

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

    // 0.4 is the pos for backboard contact (max)
    private final double[] extendServoPos = {0, 0.2, 0.3, 0.4};

    private int extendLvl = 80; // should be dividable by the number of pos

    private boolean hangMode = false;

    public Delivery(HardwareMap hardwareMap) {
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
    }

    //-------------------------------------------------------------------------------------
    //                                   AutoRed Functions
    //-------------------------------------------------------------------------------------
    public boolean autoDropPixels(double targetPos){
        dropServo.setPosition(targetPos);
        return dropServo.getPosition() == targetPos;
    }

    public boolean driveLiftToPosition(int target, int milli){
        if (getLift1Position() < target - 5 || getLift1Position() > target + 5 && milli < 2200)
            autoDriveLift((target - getLift1Position()) * -.008);
        else
            autoDriveLift(0);
        return getLift1Position() < target - 25 && getLift1Position() > target + 25;
    }

    public void autoExtend(double position){
        extendServo.setPosition(position);
    }

    public void autoDriveLift(double power){
        if (getTouchSensor1Status() && power > 0)
            liftMotor1.setPower(0);
        else
            liftMotor1.setPower(power);

        if (getTouchSensor2Status() && power > 0)
            liftMotor2.setPower(0);
        else
            liftMotor2.setPower(power);
    }



        //-------------------------------------------------------------------------------------
        //                                   Lift Functions
        //-------------------------------------------------------------------------------------

        public void driveLift ( double power){
            touchSensorEncoderReset();
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (!hangMode) {
                if (getLift1Position() <= 0 && power > 0) {
                    liftMotor1.setPower(0);
                } else {
                    liftMotor1.setPower(power);
                }

                if (getLift2Position() <= 0 && power > 0) {
                    liftMotor2.setPower(0);
                } else {
                    // this is the power multiplier, lift2 is faster... too fucking fast
                    liftMotor2.setPower(power * 0.35);
                }
            }
        }

        public void hangLift ( double power1, double power2){
            if (hangMode) {
                liftMotor1.setPower(power1);
                liftMotor2.setPower(power2);
            }
        }

        //-------------------------------------------------------------------------------------
        //                                   Outtake Functions
        //-------------------------------------------------------------------------------------

        public void changeExtensionLength ( boolean decrease, boolean increase){
            if (decrease) {
                extendLvl -= 1;
            }
            if (increase) {
                extendLvl += 1;
            }
            setExtensionPosition();
        }

        public void changeDropPosition ( boolean pressing){
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

        public boolean getTouchSensor1Status () {
            return touchSensor1.isPressed();
        }

        public void setHangMode ( boolean pressed){
            if (pressed) {
                hangMode = !hangMode;
            }
        }

        public boolean getTouchSensor2Status () {
            return touchSensor2.isPressed();
        }

        public boolean getHangModeStatus () {
            return hangMode;
        }

        public double getLift1Position () {
            return -liftMotor1.getCurrentPosition();
        }

        public double getLift2Position () {
            return liftMotor2.getCurrentPosition();
        }

        public double getExtensionPosition () {
            return extendServo.getPosition();
        }

        private void setExtensionPosition () {
            extendServo.setPosition(extendServoPos[Math.abs(extendLvl % extendServoPos.length)]);
        }

        public double getDropPosition () {
            return dropServo.getPosition();
        }
}