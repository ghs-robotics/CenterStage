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
     * @param milliseconds is the current amount of time that has passed
     * @return false-> passed time is less than timeLim, true-> passed time is greater than timeLim
     *
     * Skye possesses the robot like her dog and runs the intake in auto
     */
    public boolean autoRunIntake(double milliseconds) {
        int timeLim = 1000;
        pixelIn(milliseconds < timeLim);

        if (milliseconds < timeLim)
            return false;
        else
            return true;
    }

    /**
     * @param targetLevel is the goal position for the intake
     *
     * Jett aggressively updrafts the intake until it reaches the desired pos
     */
    public void setLiftHeight(int targetLevel){
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

