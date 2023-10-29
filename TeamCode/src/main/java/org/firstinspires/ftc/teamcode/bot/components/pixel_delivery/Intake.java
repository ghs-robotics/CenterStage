package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor conveyorBeltMotor;
    private DcMotor intakeMotor;
    private Servo intakeServo;

    double[] pos = {0.01, 0.06, 0.09, 0.13, 0.16, 0.2};

    int intakeLvl = 60;

    public Intake (HardwareMap hardwareMap) {
        conveyorBeltMotor = hardwareMap.get(DcMotor.class,"belt");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakem");
        intakeServo = hardwareMap.get(Servo.class, "intakes");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPosition(0.05);
    }

    public void pixelIn (boolean pressing) {
        if (pressing) {
            intakeMotor.setPower(0.5);
            conveyorBeltMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
            conveyorBeltMotor.setPower(0);
        }
    }

    public boolean autoRunIntake(double milliseconds) {
        int timeLim = 1000;
        pixelIn(milliseconds < timeLim);

        if (milliseconds < timeLim)
            return false;
        else
            return true;
    }

    public void setIntakeHeight(int targetLevel){
        int diff = targetLevel - getIntakeLvl();
        intakeLvl += diff;
        setHeight();
    }

    public void changeIntakeHeight(boolean decrease, boolean increase) {
        if (decrease) {
            intakeLvl -= 1;
        }
        if (increase) {
            intakeLvl += 1;
        }
        setHeight();
    }

    public int getIntakePos () {
        return getIntakeLvl();
    }

    private void setHeight () {
        intakeServo.setPosition(pos[Math.abs(intakeLvl % pos.length)]);
    }

    private int getIntakeLvl(){
        return Math.abs(intakeLvl % pos.length);
    }
}

