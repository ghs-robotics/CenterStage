package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake {
    private DcMotor conveyorBeltMotor;
    private DcMotor intakeMotor;
    private Servo intakeServo;

    double[] pos = {0.01, 0.05, 0.08, 0.12, 0.15, 0.2};

    int intakeLvl = 60;

    public Intake (HardwareMap hardwareMap) {
        conveyorBeltMotor = hardwareMap.get(DcMotor.class,"belt");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakem");
        intakeServo = hardwareMap.get(Servo.class, "intakes");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPosition(0.05);
    }

    /**
     * ejects pixel
     */
    public void autoPixelOut () {
        intakeMotor.setPower(-0.5);
        conveyorBeltMotor.setPower(-1);
    }

    /**
     * @param pressing (boolean) runs the intake when pressing, stops intake when !pressing
     *
     * wingman spins the Gekko wheels to intake each pixel
     */

    public void pixelIn (boolean pressing) {
        if (pressing) {
            intakeMotor.setPower(0.5);
            conveyorBeltMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
            conveyorBeltMotor.setPower(0);
        }
    }

    /**
     * intakes pixel
     * @param power desired power of motors
     */
    public void pixelIn (double power) {
        intakeMotor.setPower(power / 2);
        conveyorBeltMotor.setPower(power);
    }

    /**
     * @param targetLevel is the goal position for the intake
     *
     * Jett aggressively updrafts the intake until it reaches the desired pos
     */
    public void setIntakeHeight(int targetLevel){
        int diff = targetLevel - getIntakeLvl();
        intakeLvl += diff;
        setHeight();
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
     *
     * (cypher hacks into the code and changes it)
     */
    public int getIntakePos () {
        return getIntakeLvl();
    }

    /**
     * sage lifts the intake with her wall based on the intake level
     */
    private void setHeight () {
        intakeServo.setPosition(pos[Math.abs(intakeLvl % pos.length)]);
    }

    /**
     * @return current intake lvl (number between 0 and pos.length - 1)
     *
     * Brimstone checks his wrist screen to see the intakeLvl
     */
    private int getIntakeLvl(){
        return Math.abs(intakeLvl % pos.length);
    }
}

