package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    DcMotor liftMotor1;
    DcMotor liftMotor2;

    public static final int MIN = 0;
    public static final int MAX = 100;
    public static final int LOW = 20;
    public static final int MID = 40;
    public static final int HIGH = 60;

//    public Lift (HardwareMap hardwareMap, Telemetry telemetry) {
//        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
//        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
//        telemetry.update();
//    }

    public void raiseLift () {
        if (liftMotor1.getCurrentPosition() >= MIN && liftMotor1.getCurrentPosition() <= MAX) {
            liftMotor1.setPower(1);
        }
    }

    public void moveToLow () {
        liftMotor1.setTargetPosition(LOW);
    }

    public void moveToMid () {
        liftMotor1.setTargetPosition(MID);
    }

    public void moveToHigh () {
        liftMotor1.setTargetPosition(HIGH);
    }
}
