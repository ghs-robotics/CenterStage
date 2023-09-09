package org.firstinspires.ftc.teamcode.robot.components.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallDrive implements Drivebase {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor backDrive;

    private double lp;
    private double rp;
    private double bp;


    public BallDrive(HardwareMap hardwareMap) {

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        backDrive = hardwareMap.get(DcMotor.class, "back");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void calculateDrivePowers(double x, double y, double rot){
        bp = x;
        lp = y - rot;
        rp = y + rot;

        setMotorPowers();
    }

    @Override
    public int[] getEncoderTicks() {
        int[] ticks = {leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(),
                backDrive.getCurrentPosition()};
        return ticks;
    }

    private void setMotorPowers(){
        leftDrive.setPower(lp);
        rightDrive.setPower(rp);
        backDrive.setPower(bp);
    }
}
