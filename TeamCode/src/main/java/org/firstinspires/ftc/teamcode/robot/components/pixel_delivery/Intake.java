package org.firstinspires.ftc.teamcode.robot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// todo needs renaming
public class Intake {
    DcMotor intakeMotor;

    public Intake (HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        telemetry.update();
    }
    public void spinIntake () {
        intakeMotor.setPower(1.0);
    }
}
