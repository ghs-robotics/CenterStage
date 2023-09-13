package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// todo needs renaming
public class Intake {
    private DcMotor intakeMotor;
    private Servo intakeServo;

    // TODO: make one position for each pixel (5 in each stack) (2 more)
    double[] pos = {0.01, 0.07, 0.1};

    int intakeLvl = 60;

    public Intake (HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
    }

    public void pixelIn (boolean press) {
        if (press) {
            intakeMotor.setPower(0.5);
        } else {
            intakeMotor.setPower(0);
        }
    }

    private void setHeight () {
        intakeServo.setPosition(pos[Math.abs(intakeLvl % pos.length)]);
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

    public double getServoPos () {
        return intakeServo.getPosition();
    }
}
