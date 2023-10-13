package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private Servo intakeServo;

    double[] pos = {0.01, 0.06, 0.09, 0.13, 0.16, 0.2};

    int intakeLvl = 60;

    public Intake (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * @param pressing (boolean) runs the intake when pressing, stops intake when !pressing
     *
     * wingman spins the Gekko wheels to intake each pixel
     */

    public void pixelIn (boolean pressing) {
        if (pressing) {
            intakeMotor.setPower(0.5);
        } else {
            intakeMotor.setPower(0);
        }
    }

    /**
     * @param decrease (boolean) decreases the intake level
     * @param increase (boolean) increases the intake level
     *
     * killjoy adjusts the intake level based on the parameter
     * it also calls the setHeight() to update the level and lift the intake
     */

    public void changeIntakeHeight(boolean decrease, boolean increase) {
        if (decrease) {
            intakeLvl -= 1;
        }
        if (increase) {
            intakeLvl += 1;
        }
        setHeight();
    }

    /**
     * @return the intake position for telemetry
     * (cypher hacks into the code and changes it)
     */

    public int getIntakePos () {
        return Math.abs(intakeLvl % pos.length);
    }

    /**
     * sage lifts the intake with her wall based on the intake level
     */

    private void setHeight () {
        intakeServo.setPosition(pos[Math.abs(intakeLvl % pos.length)]);
    }
}

