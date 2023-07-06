package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallDrive {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor backDrive;

    private double lp;
    private double rp;
    private double bp;

    Telemetry telemetry;

    public BallDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        backDrive = hardwareMap.get(DcMotor.class, "back");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void calculateDrivePowers(double x, double y, double rot){
        bp = x;
        lp = y - rot;
        rp = y + rot;

        setMotorPowers();
    }

    private void setMotorPowers(){
        leftDrive.setPower(lp);
        rightDrive.setPower(rp);
        backDrive.setPower(bp);
    }
}
