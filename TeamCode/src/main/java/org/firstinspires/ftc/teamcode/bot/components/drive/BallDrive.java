package org.firstinspires.ftc.teamcode.bot.components.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.components.Gyro;

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

    private PID xPID;
    private PID yPID;

    public double errorX;

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

        xPID = new PID();
        yPID = new PID();

        lastL = 0;
        lastR = 0;
        lastB = 0;

        this.gyro = gyro;
    }

    public boolean runToPosition(double x, double y){
        xPID.setTarget(x);
        yPID.setTarget(y);

        double xPower = xPID.getOutput(x);
        double yPower = yPID.getOutput(y);

        calculateDrivePowers(xPower, yPower, 0, true);

        return xPID.getError() + yPID.getError() < 100;
    }


    public void update(){
        updateValues();

        double arc = (deltaR + deltaL) * .5;

        x += arc * Math.cos(heading + deltaH *.5);
        y += arc * Math.sin(heading + deltaH *.5);
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

    public void resetEncoders() {
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

    private class PID{
        private double lastError;
        private double lastIntegral;

        private double error;
        private double integral;
        private double derivative;

        private double kp = .6;
        private double ki = 1.2;
        private double kd = 0.05;
        private double bias = 0;

        private double target;

        private ElapsedTime timer;

        private double lastIterationTime;

        PID(double p, double i, double d){
            this();
            kp = p;
            ki = i;
            kd = d;
        }

        PID(){
            timer = new ElapsedTime();
        }

        double getOutput(double current){
            double iterationTime = timer.milliseconds() - lastIterationTime;
            lastIterationTime += iterationTime;

            error = target - current;
            integral = lastIntegral + error * iterationTime;
            derivative = (error - lastError) / iterationTime;

            double out = kp * error + ki * integral + kd * derivative + bias;

            lastError = error;
            lastIntegral = integral;

            return out;
        }

        double getError(){
            return error;
        }

        void setTarget(double target){
            if (this.target == target)
                return;

            this.target = target;
            lastError = 0;
            lastIntegral = 0;
            error = 0;
            integral = 0;
            derivative = 0;
            timer.reset();
            lastIterationTime = 0;
        }

    }
}
