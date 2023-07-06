package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallDrive {
    int x;
    int y;
    int heading;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor backDrive;

    public BallDrive(HardwareMap hardwareMap, Telemetry telemetry) {

    }
}
