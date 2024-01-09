package org.firstinspires.ftc.teamcode.bot.components.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.control.PID;

public class BallDrive {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor backDrive;

    private double lp;
    private double rp;
    private double bp;

    private Gyro gyro;

    private boolean metaDriveOn;

    private double x;
    private double y;
    private double heading;

    private double lastL, lastR, lastB;
    private double left, right, back;
    private double deltaL, deltaR, deltaB, deltaH;
    private double lastH;

    private final double MM_PER_TICK = (35 * Math.PI) / 8192;

    private PID xPID;
    private PID yPID;

    public double resetCounter;
    public double error;

    public BallDrive(HardwareMap hardwareMap, Gyro gyro) {

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        backDrive = hardwareMap.get(DcMotor.class, "back");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE); // y-axis encoder
        backDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD); // x-axis encoder

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        xPID = new PID(.05, .2, .01);
        yPID = new PID(.05, .2, .01);

        lastL = 0;
        lastR = 0;
        lastB = 0;

        this.gyro = gyro;
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

        this.y += MM_PER_TICK * dy;
        this.x += MM_PER_TICK * dx;

    }

    public boolean runToPosition(double targetX, double targetY){
        xPID.setTarget(targetX);
        yPID.setTarget(targetY);

        double xPower = xPID.getOutput(this.x);
        double yPower = yPID.getOutput(this.y);

        // TODO Incorporate PID
//        if (Math.abs(xPower) > .05)
//            calculateDrivePowers(xPower, 0, 0, true);
//        else
//            calculateDrivePowers(0, yPower, 0, true);

//        calculateDrivePowers(xPower, yPower, 0, true);
        return Math.abs(xPID.getError()) + Math.abs(yPID.getError()) < 5;
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

    public void resetCoords(){
        resetEncoders();
        x = 0;
        y = 0;
        heading = 0;

        resetCounter++;
    }

    public double getXError(){
        return xPID.getOutput(x);
    }

    public double getYError(){
        return yPID.getOutput(y);
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
