package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.PIDControllers.NavigationPID;

public class BallDrive {
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final DcMotor backDrive;

    private double lp;
    private double rp;
    private double bp;

    private final Gyro gyro;

    private boolean metaDriveOn;

    private double x;
    private double y;
    private double heading;

    // the following variables are only for the use
    private double lastL, lastR, lastB, lastH;
    private double left, right, back;
    private double deltaL, deltaR, deltaB, deltaH;

    private double lastXError;
    private double lastYError;

    private final double MM_PER_TICK = (35 * Math.PI) / 8192;

    public BallDrive(HardwareMap hardwareMap, Gyro gyro) {
        this.gyro = gyro;

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        backDrive = hardwareMap.get(DcMotor.class, "back");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastL = 0;
        lastR = 0;
        lastB = 0;
    }

    public void update(){
        updateValues();
        updatePosition();
    }

    private void updatePosition(){
        double ds = (deltaR + deltaL) * .5;

        double dx1 = ds * Math.cos(deltaH / 2);
        double dy1 = ds * Math.sin(deltaH / 2);

        double dx2 = deltaB * Math.sin(deltaH / 2);
        double dy2 = deltaB * Math.cos(deltaH / 2);

        double dy = dx1 * Math.cos(heading) + dx2 * Math.sin(heading);
        double dx = - dy1 * Math.sin(heading) + dy2 * Math.cos(heading);

        this.y = this.y + MM_PER_TICK * dy;
        this.x = this.x + MM_PER_TICK * dx;
    }

    public boolean runToPosition(NavigationPID xPID, NavigationPID yPID){
        double xPower = -xPID.getOutput(this.x);
        double yPower = yPID.getOutput(this.y);

        if (Math.abs(lastXError - xPID.getError()) < 3)
            xPower = 0;

        if (Math.abs(lastYError - yPID.getError()) < 3)
            yPower = 0;

        calculateDrivePowers(xPower, yPower, 0, true);

        lastXError = xPID.getError();
        lastYError = yPID.getError();

        return Math.abs(xPID.getError()) + Math.abs(yPID.getError()) < 6 || xPower + yPower == 1;
    }

    public void calculateDrivePowers(double x, double y, double rot) {
        bp = x;
        lp = y - rot;
        rp = y + rot;

        setMotorPowers();
    }

    public void calculateDrivePowers(double x, double y, double rot, boolean driveMode){
        metaDriveOn = driveMode;

        double driveX = x;
        double driveY = y;

        if(driveMode) {
            driveX = x * Math.cos(heading) - y * Math.sin(heading) ;
            driveY = y * Math.cos(heading) + x * Math.sin(heading);
        }
        calculateDrivePowers(driveX, driveY, rot);
    }

    public void resetCoords(){
        resetEncoders();
        x = 0;
        y = 0;
        heading = 0;
    }

    private void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean getDriveMode() {
        return metaDriveOn;
    }

    private void setMotorPowers(){
        leftDrive.setPower(lp);
        rightDrive.setPower(rp);
        backDrive.setPower(bp);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return Math.toDegrees(heading);
    }

    private void updateValues(){
        back = backDrive.getCurrentPosition();
        left = leftDrive.getCurrentPosition();
        right = rightDrive.getCurrentPosition();
        heading = gyro.getHeading();

        deltaB = back - lastB;
        deltaL = left - lastL;
        deltaR = right - lastR;
        deltaH = heading - lastH;

        lastB = back;
        lastL = left;
        lastR = right;
        lastH = heading;
    }
}