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

    private double x;
    private double y;
    private double heading;

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

    public void update(){
        // todo: needs updating
        double lTracking = 153.275;
        double rTracking = 164.109;
        double bTracking = 80;

        int left = leftDrive.getCurrentPosition();
        int right = rightDrive.getCurrentPosition();
        int back = backDrive.getCurrentPosition();

        heading = (left - right) / (lTracking - rTracking);

        double y = 2 * (right / heading + rTracking) * Math.sin(heading / 2);
        double x = 2 * (back / heading + bTracking) * Math.sin(heading / 2);

        this.x = x;
        this.y = y;
    }

    public boolean runToPosition(int tarX, int tarY){
        return runToPosition(tarX, tarY, 0);
    }

    public boolean runToPosition(int tarX, int tarY, double tarHeading){
        int maxError = 100; // should be in ticks
        double headingError = 30;
        double slowingRangeMultiplier = 1.5;

        double xPower = 0;
        double yPower = 0;
        double rotPower = 0;

        if (Math.abs(tarX - x) > maxError)
            xPower = (tarX - x) / (maxError * slowingRangeMultiplier);
        else if (Math.abs(tarY - y) > maxError)
            yPower = -(tarY - y) / (maxError * slowingRangeMultiplier);
        else if (Math.abs(tarHeading - heading) > headingError){
            rotPower = (tarHeading - heading) / (headingError * slowingRangeMultiplier);
        }
        else
            return true;

        calculateDrivePowers(xPower, yPower, rotPower, true);
        return false;
    }


    @Override
    public void calculateDrivePowers(double x, double y, double rot) {
        bp = x;
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
        int[] ticks = {leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), backDrive.getCurrentPosition()};
        return ticks;
    }

    @Override
    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public boolean getDriveMode() {
        return metaDriveOn;
    }

    @Override
    public DcMotorEx[] getEncoderMotors(){
        return new DcMotorEx[]{leftDrive, rightDrive, backDrive};

    }

    private void setMotorPowers(){
        leftDrive.setPower(lp);
        rightDrive.setPower(rp);
        backDrive.setPower(bp);
    }

    public void resetCoords(){
        x = 0;
        y = 0;
        heading = 0;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
