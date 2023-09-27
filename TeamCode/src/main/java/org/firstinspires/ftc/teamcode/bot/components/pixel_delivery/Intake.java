package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// todo needs renaming
public class Intake {
    private DcMotor intakeMotor;
    private Servo intakeServo;

    // TODO: make one position for each pixel (5 in each stack) (2 more)
    double[] pos = {0.01, 0.06, 0.09, 0.13, 0.16, 0.2};

    int intakeLvl = 60;

    public Intake (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void pixelIn (boolean pressing) {
        if (pressing) {
            intakeMotor.setPower(0.5);
        } else {
            intakeMotor.setPower(0);
        }
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
        return Math.abs(intakeLvl % pos.length);
    }

    private void setHeight () {
        intakeServo.setPosition(pos[Math.abs(intakeLvl % pos.length)]);
    }
}

