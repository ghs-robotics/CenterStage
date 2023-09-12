package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import android.graphics.HardwareRenderer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// todo needs renaming
public class Intake {
    DcMotor intakeMotor;
    Servo intakeServo;



    public Intake (HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        telemetry.update();
    }

    public void pixelIn (boolean press) {
        if (press) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setPower(0.5);
        } else {
            intakeMotor.setPower(0);
        }
    }
}
