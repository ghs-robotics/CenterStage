package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    //-------------------------------------------------------------------------------------
    //                                   Auto Functions
    //-------------------------------------------------------------------------------------

    public void autoPixelOut () {
        intakeMotor.setPower(-0.5);
        conveyorBeltMotor.setPower(-1);
    }

    public void setIntakePosition(int targetLevel){
        // i literally do not know what to place there (where the 0 is)
        // the zero is a placeholder. it is an auto function
        // you can fix it hehe
        int diff = targetLevel - 0;
        intakeServo.setPosition(pos[Math.abs(intakeLvl % pos.length)]);
    }

    //-------------------------------------------------------------------------------------
    //                                   Intake Functions
    //-------------------------------------------------------------------------------------

    public void pixelIn (double power) {
        intakeMotor.setPower(power / 2);
        conveyorBeltMotor.setPower(power);
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

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    private void setIntakePosition() {
        intakeServo.setPosition(Math.abs(intakeLvl % pos.length));
    }

    public double getIntakePosition () {
        return intakeServo.getPosition();
    }
}

