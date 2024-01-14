package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public double resetCounter;
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

        this.y = this.y + MM_PER_TICK * dy;
        this.x = this.x + MM_PER_TICK * dx;
    }

    public boolean runToPosition(PID xPID, PID yPID){
        double xPower = -xPID.getOutput(this.x);
        double yPower = yPID.getOutput(this.y);

        // TODO Incorporate PID
//        if (Math.abs(xPower) > .05)
//            calculateDrivePowers(xPower, 0, 0, true);
//        else
//            calculateDrivePowers(0, yPower, 0, true);

        calculateDrivePowers(xPower, yPower, 0, true);

        errorX = xPID.getError();

        return Math.abs(xPID.getError()) + Math.abs(yPID.getError()) < 2;
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
        return errorX;
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


//    private class PID{
//        private double lastError;
//        private double lastIntegral;
//
//        private double error;
//        private double integral;
//        private double derivative;
//
//        private double kp = .6;
//        private double ki = 1.2;
//        private double kd = 0.05;
//        private double bias = 0;
//
//        private double target;
//
//        private ElapsedTime timer;
//
//        private double lastIterationTime;
//
//        private double maxIntegral;
//
//        PID(double p, double i, double d){
//            this();
//            kp = p;
//            ki = i;
//            kd = d;
//        }
//
//        PID(){
//            timer = new ElapsedTime();
//        }
//
//        double getOutput(double current){
//            double iterationTime = timer.milliseconds() - lastIterationTime;
//            lastIterationTime += iterationTime;
//
//            error = target - current;
//            integral = lastIntegral + error * iterationTime;
//            derivative = (error - lastError) / iterationTime;
//
//            constrain();
//
//            double out = (kp * error) + (ki * integral) + (kd * derivative) + bias;
//
//            lastError = error;
//            lastIntegral = integral;
//
//            return out;
//        }
//
//        double getError(){
//            return error;
//        }
//
//        void setMaxError(double max){
//            maxIntegral = max;
//        }
//
//        void constrain(){
//            if (Math.abs(integral) > maxIntegral)
//                integral = maxIntegral * (integral / Math.abs(integral));
//        }
//
//        void setTarget(double target){
//            if (this.target == target)
//                return;
//
//            this.target = target;
//            lastError = 0;
//            lastIntegral = 0;
//            error = 0;
//            integral = 0;
//            derivative = 0;
//            timer.reset();
//            lastIterationTime = 0;
//        }
//
//    }

}
