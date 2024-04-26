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

    // for recording the last positions of the robot
    private double lastL, lastR, lastB, lastH;
    // for he current positions
    private double left, right, back;
    // the calculated change between current and last state
    private double deltaL, deltaR, deltaB, deltaH;

    // constant multiplier to convert ticks to a usable unit (in this case MM but its a little off)
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

    /**
     * helper method that does the math to track how far the robot has travelled since the last
     * recorded position.
     */
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

    /**
     * @param xPID the x-axis pid controller that already has a set target position.
     * @param yPID the y-axis pid controller that already has a set target position.
     * @return whether or not we have arrived within an acceptable range of the target position
     */
    public boolean runToPosition(NavigationPID xPID, NavigationPID yPID){
        double xPower = -xPID.getOutput(this.x);
        double yPower = yPID.getOutput(this.y);

//        if (Math.abs(lastXError - xPID.getError()) < 1)
//            xPower = 0;
//
//        if (Math.abs(lastYError - yPID.getError()) < 1)
//            yPower = 0;

        calculateDrivePowers(xPower, yPower, 0, true);

//        double lastXError = xPID.getError();
//        double lastYError = yPID.getError();

        return Math.abs(xPID.getError()) + Math.abs(yPID.getError()) < 6 || xPower + yPower == 1;
    }

    /**
     * @param x how much you want the robot to strafe (drive sideways)
     * @param y how much you want the robot to drive forward and backward
     * @param rot how much you want the robot to rotate
     */
    public void calculateDrivePowers(double x, double y, double rot) {
        bp = x;
        lp = y - rot;
        rp = y + rot;

        setMotorPowers();
    }

    /**
     * In 4042 field centric drive is known as meta drive for some reason. :/
     * Field-centric drive uses the field to define the x and y axis instead of the robot. When using
     * meta drive y axis is always going to be the initial direction of the robot, no matter what
     * orientation the robot ends up in.
     *
     * @param x how much you want the robot to strafe (drive sideways)
     * @param y how much you want the robot to drive forward and backward
     * @param rot how much you want the robot to rotate
     * @param driveMode whether or not the robot is in meta drive
     */
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