package org.firstinspires.ftc.teamcode.bot.components.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bot.components.Gyro;

public class BallDrive implements Drivebase {
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotorEx backDrive;

    private double lp;
    private double rp;
    private double bp;

    private Gyro gyro;

    private boolean metaDriveOn;

    public BallDrive(HardwareMap hardwareMap, Gyro gyro) {

        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
        backDrive = hardwareMap.get(DcMotorEx.class, "back");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE); // y-axis encoder
        backDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD); // x-axis encoder

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gyro = gyro;
    }

    @Override
    public void calculateDrivePowers(double x, double y, double rot) {
        bp = x + rot;
        lp = y - rot;
        rp = y + rot;

        setMotorPowers();
    }

    @Override
    public void calculateDrivePowers(double x, double y, double rot, boolean driveMode){
        double angle = gyro.getHeading();
        metaDriveOn = driveMode;

        double driveX = x;
        double driveY = y;

        if(driveMode) {
            driveX = x * Math.cos(angle) - y * Math.sin(angle) ;
            driveY = y * Math.cos(angle) + x * Math.sin(angle);
        }
        calculateDrivePowers(driveX, driveY, rot);
    }

    @Override
    public int[] getEncoderTicks() {
        // 0 gets y, 1 gets x
        int[] ticks = {leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition()};
        return ticks;
    }

    @Override
    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public boolean getDriveMode() {
        return metaDriveOn;
    }

    @Override
    public DcMotorEx[] getEncoderMotors(){
        return new DcMotorEx[]{leftDrive, rightDrive};

    }

    private void setMotorPowers(){
        leftDrive.setPower(lp);
        rightDrive.setPower(rp);
        backDrive.setPower(bp);
    }
}
