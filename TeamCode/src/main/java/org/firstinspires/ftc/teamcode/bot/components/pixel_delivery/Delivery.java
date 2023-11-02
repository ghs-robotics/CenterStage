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

    private int liftLvl = 60;
    private int dropLvl = 60;

    private boolean runLiftToPosition;

    public Delivery (HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift2");
        extensionServo = hardwareMap.get(CRServo.class, "extend");
        droppingServo = hardwareMap.get(Servo.class, "drop");

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionServo.setDirection(DcMotorSimple.Direction.REVERSE);
        droppingServo.setPosition(0);

        runLiftToPosition = false;
    }

    private void setDeliveryPositions() {
        liftMotor1.setTargetPosition(liftMotorPos[Math.abs(liftLvl % liftMotorPos.length)]);
        droppingServo.setPosition(dropServoPos[Math.abs(dropLvl % dropServoPos.length)]);
    }

    //-------------------------------------------------------------------------------------
    //                                   Lift Functions
    //-------------------------------------------------------------------------------------

    public void driveLift (double power) {
        if (!runLiftToPosition) {
            liftMotor1.setPower(power);
            liftMotor2.setPower(power);
        }
    }

    public void setRunLiftToPosition(boolean button){
        if (button)
            runLiftToPosition = !runLiftToPosition;

        runLiftToPosition();
    }

    private void runLiftToPosition(){
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setPower(liftMotor1.getPower());
    }

    public void changeLiftHeight (boolean increase) {
        if (increase) {
            liftLvl += 1;
        }
        setDeliveryPositions();
    }

    public int getLiftPosition() {
        return liftMotor1.getCurrentPosition();
    }

    public void resetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //-------------------------------------------------------------------------------------
    //                                   Drop Functions
    //-------------------------------------------------------------------------------------

    public void changeDropPosition (boolean increase) {
        if (increase) {
            dropLvl += 1;
        }
        setDeliveryPositions();
    }

    public double getDropPosition () {
        return droppingServo.getPosition();
    }

    //-------------------------------------------------------------------------------------
    //                                   Outtake Functions
    //-------------------------------------------------------------------------------------

    public void extendOuttake (double power) {
        extensionServo.setPower(power);
    }
}
