package org.firstinspires.ftc.teamcode.robot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// todo needs renaming
public class Lift {
    public static final int MIN = 0;
    public static final int MAX = 1000;
    public static final int LOW = 200;
    public static final int MID = 400;
    public static final int HIGH = 600;

    DcMotor liftMotor1;
    DcMotor liftMotor2;

    public Lift (HardwareMap hardwareMap, Telemetry telemetry) {
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        telemetry.update();
    }

    public void raiseLift (double lift) {
        if (liftMotor1.getCurrentPosition() >= MIN && liftMotor1.getCurrentPosition() <= MAX) {
            liftMotor1.setPower(lift);
            liftMotor2.setPower(lift);
        }
    }

    public void setLow () {
        liftMotor1.setTargetPosition(LOW);
        liftMotor2.setTargetPosition(LOW);
    }

    public void setMid () {
        liftMotor1.setTargetPosition(MID);
        liftMotor2.setTargetPosition(MID);
    }

    public void setHigh () {
        liftMotor1.setTargetPosition(HIGH);
        liftMotor2.setTargetPosition(HIGH);
    }
}

