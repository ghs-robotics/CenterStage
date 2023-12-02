package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Delivery {
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    private Servo extendServo;
    private Servo dropServo;

    private TouchSensor touchSensor;

    private double[] extendServoPos = {0};

    private int extendLvl = 60;

    public static final double DROPPER_INTAKING = 0.1;
    public static final double DROPPER_FIRST = 0.5;
    public static final double DROPPER_SECOND = 0.6;

    private double sentPower;

    /**
     * adds all the variables to robot class, sets dropping servo to 0
     * @param hardwareMap is how the code interacts with the robot
     */
    public Delivery (HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift2");
        extendServo = hardwareMap.get(Servo.class, "extend");
        dropServo = hardwareMap.get(Servo.class, "drop");

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE); // currently polarity is reversed
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //-------------------------------------------------------------------------------------
    //                                   Auto Functions
    //-------------------------------------------------------------------------------------

    /**
     * runs extension, stops when elapsed time is over 700ms
     * @param dir direction
     * @param curMillisecond time
     * @return while running if less than 700 is false, after 700 its true and stops running
     */
//    public boolean autoRunExtension(double dir, double curMillisecond){
//        if (curMillisecond < 550){
//            setExtensionPower(dir);
//        }else
//            setExtensionPower(0);
//        return curMillisecond > 700;
//    }

    /**
     * drops pixel
     * @param targetPos position servo needs to be to drop pixels
     * @return  is false and keeps going until reaches position, then true and stops
     */
    public boolean autoDropPixels(double targetPos){
        dropServo.setPosition(targetPos);
        return dropServo.getPosition() == targetPos;
    }

    /**
     * sets the lift position
     */
//    public void setLiftPosition() {
//        liftMotor1.setTargetPosition(liftMotorPos[Math.abs(liftLvl % liftMotorPos.length)]);
//    }

    /**
     * raises lift to desired position
     * @param target the desired point
     * @return if position less than desired, is false, keeps running, and if higher, then true and stops
     */
//    public boolean driveLiftToPosition(int target){
//        liftLvl = target;
//        target = liftMotorPos[getLiftLvl()];
//
//        if (getLiftPosition() < target - 25 || getLiftPosition() > target + 25)
//            driveLift((getLiftPosition() - target) / 350.0);
//        else
//            driveLift(-0.1);
//        return getLiftPosition() < target - 25 || getLiftPosition() > target + 25;
//    }

    //-------------------------------------------------------------------------------------
    //                                   Lift Functions
    //-------------------------------------------------------------------------------------

    /**
     * sets lift motor power
     * @param power how much power is wanted
     */
    public void driveLift (double power) {
        touchSensorEncoderReset();
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sentPower = power;

        if (Math.abs(power) > 0.1) {
            limitLift(power);
        } else if (Math.abs(power) < 0.1) {
            setLiftPower(0);
        }
    }

    // next two functions for TuneLift OpMode
    public void driveLiftMotor1 (double power) {
        touchSensorEncoderReset();
        liftMotor1.setPower(power);
    }

    public void driveLiftMotor2 (double power) {
        liftMotor2.setPower(power);
    }

    public void touchSensorEncoderReset () {
        if (getTouchSensorStatus()) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * limits lift position and power
     * @param power wanted amount of power
     */
    private void limitLift(double power){
        int limit = 1430;

        if (getLiftPosition() > limit && power > 0) {
            power = 0;
        } else if (getLiftPosition() > limit - 150){
            power *= (limit - getLiftPosition()) / 200.0;
        } else if (getLiftPosition() < 0 && power > 0){
            power = 0;
        }
        setLiftPower(power);
    }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    public void changeExtensionLength (boolean decrease, boolean increase) {
        if (decrease) {
            extendLvl -= 1;
        }
        if (increase) {
            extendLvl += 1;
        }
    }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    /**
     * @param power sets the power of both motors on the lift
     */
    private void setLiftPower(double power){
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    /**
     * resets encoders
     */
    public void resetEncoders () {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getLiftPosition () {
        return -liftMotor1.getCurrentPosition();
    }

    public void setExtensionPosition (boolean pressed) {
        if (pressed) {
            extendLvl += 1;
        }

    }

    public double getExtensionPosition () {
        return extendServo.getPosition();
    }

    public void setDropPosition (boolean released) {
        if (released) {
            dropServo.setPosition(0.2);
        } else {
            dropServo.setPosition(0);
        }
    }

    public double getDropPosition () {
        return dropServo.getPosition();
    }

    public boolean getTouchSensorStatus () {
        return touchSensor.isPressed();
    }
}
