package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor conveyorBeltMotor;
    private final DcMotor intakeMotor;
    private final Servo intakeServo;
    private final double[] intakeServoPos = {0.01, 0.07, 0.12, 0.14, 0.18, 0.2};

    private int intakeLvl = 60;

    public Intake(HardwareMap hardwareMap) {
        conveyorBeltMotor = hardwareMap.get(DcMotor.class, "belt");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakem");
        intakeServo = hardwareMap.get(Servo.class, "intakes");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPosition(0.05);
    }

    //-------------------------------------------------------------------------------------
    //                                   AutoRed Functions
    //-------------------------------------------------------------------------------------

    public void autoPixelOut() {
        intakeMotor.setPower(-0.5);
        conveyorBeltMotor.setPower(-1);
    }

    public void setIntakeHeight(int targetLevel) {
        int diff = targetLevel - getIntakeLvl();
        intakeLvl += diff;
        setIntakePosition();
    }

    //-------------------------------------------------------------------------------------
    //                                   Intake Functions
    //-------------------------------------------------------------------------------------

    public void pixelIn(double power) {
        intakeMotor.setPower(power / 2);
        conveyorBeltMotor.setPower(-power);
    }

    public void changeIntakeHeight(boolean decrease, boolean increase) {
        if (decrease) {
            intakeLvl -= 1;
        }
        if (increase) {
            intakeLvl += 1;
        }
        setIntakePosition();
    }

    public int getIntakePower() {
        int power = 0;
        if (conveyorBeltMotor.getPower() > 0) {
            power = -1;
        } else if (conveyorBeltMotor.getPower() < 0) {
            power = 1;
        }
        return power;
    }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    private void setIntakePosition () {
        intakeServo.setPosition(intakeServoPos[Math.abs(intakeLvl % intakeServoPos.length)]);
    }

    public double getIntakePosition () {
        return intakeServo.getPosition();
    }

    private int getIntakeLvl () {
        return Math.abs(intakeLvl % intakeServoPos.length);
    }
}