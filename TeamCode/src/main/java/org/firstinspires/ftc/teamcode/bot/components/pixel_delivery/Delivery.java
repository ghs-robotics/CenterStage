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
    private double[] dropServoPos = {0.1, 0.5, 0.6};

    public static final double DROPPER_INTAKING = 0.1;
    public static final double DROPPER_FIRST = 0.5;
    public static final double DROPPER_SECOND = 0.6;

    private double sentPower;

    private int liftLvl = 60;
    private int dropLvl = 60;

    private boolean runLiftToPosition;

    public Delivery (HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift2");
        extensionServo = hardwareMap.get(CRServo.class, "extend");
        droppingServo = hardwareMap.get(Servo.class, "drop");

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE); // currently polarity is reversed
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionServo.setDirection(DcMotorSimple.Direction.REVERSE);
        droppingServo.setPosition(0.1);

        runLiftToPosition = false;
    }

    //-------------------------------------------------------------------------------------
    //                                   Auto Functions
    //-------------------------------------------------------------------------------------

    public boolean autoRunExtension(double position, double curMillisecond){
        if (curMillisecond < 550){
            setExtensionPower(position);
        }else
            setExtensionPower(0);
        return curMillisecond > 700;
    }

    public boolean autoDropPixels(double targetPos){
        droppingServo.setPosition(targetPos);
        return droppingServo.getPosition() == targetPos;
    }

    public boolean driveLiftToPosition(int target){
        liftLvl = target;
        target = liftMotorPos[getLiftLvl()];

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

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sentPower = power;

        if (!runLiftToPosition && Math.abs(power) > 0.1) {
            limitLift(power);
        } else if (Math.abs(power) < 0.1) {
            setLiftPower(0);
        }
    }


    public void changeLiftHeight (boolean decrease, boolean increase) {
        if (decrease) {
            liftLvl -= 1;
        }
        if (increase) {
            liftLvl += 1;
        }

        if (runLiftToPosition)
            driveLiftToPosition(liftMotorPos[getLiftLvl()]);
    }

    private void limitLift(double power){
        int limit = 1430;

        if (getLiftPosition() > limit && power > 0) {
            power = 0;
        }else if (getLiftPosition() > limit - 150){
            power *= (limit - getLiftPosition()) / 200.0;
        } else if (getLiftPosition() < 0 && power > 0){
            power = 0;
        }
        setLiftPower(power);
    }

    // should only be used for the tuning teleop
    public void tuneLift(double lm1, double lm2){
        liftMotor1.setPower(lm1);
        liftMotor2.setPower(lm2);
    }

    //-------------------------------------------------------------------------------------
    //                                   Drop Functions
    //-------------------------------------------------------------------------------------

    public void changeDropPosition (boolean increase) {
        if (increase) {
            dropLvl += 1;
        }
    }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    public void setRunLiftToPosition(boolean button){
        if (button)
            runLiftToPosition = !runLiftToPosition;
    }

    /**
     * @param power sets the power of both motors on the lift
     */
    private void setLiftPower(double power){
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public void resetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getExtensionLvl(){
        return Math.abs(dropLvl % dropServoPos.length);
    }

    public void setExtensionPower(double power) {
        extensionServo.setPower(power);
    }

    public boolean getLiftMode(){
        return runLiftToPosition;
    }

    public double getDropPosition () {
        return droppingServo.getPosition();
    }

    public int getLiftLvl(){
        return Math.abs(liftLvl % liftMotorPos.length);
    }

    public int getLiftPosition() {
        return liftMotor1.getCurrentPosition();
    }
}
