package org.firstinspires.ftc.teamcode.robot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// todo needs renaming
public class Intake {
    DcMotor intakeMotor;
    public Intake (HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        telemetry.update();
    }
    public void spinRight () {
        intakeMotor.setPower(1.0);
    }
    public void spinLeft () {
        intakeMotor.setPower(-1.0);
    }
}
