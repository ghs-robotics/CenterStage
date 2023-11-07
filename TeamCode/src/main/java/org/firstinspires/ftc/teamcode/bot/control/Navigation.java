package org.firstinspires.ftc.teamcode.bot.control;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;

import com.acmerobotics.roadrunner.ftc.Encoder;

public class Navigation {
    // axis based on the robot's starting position
    private double x;
    private double y;
    private double gyroHeading;

    private int yEncoder;
    private int xEncoder;

    private int deltaX;
    private int deltaY;
    private double deltaHeading;

    private final int TICKS_PER_REV = 8192;
    private final double WHEEL_RAD = 17.5; // in mm

    private final double[] X_DIS_FROM_CENTER = new double[]{164.109, 48.88, 171.23}; // in mm
    private final double[] Y_DIS_FROM_CENTER = new double[]{153.275, 60.916, 164.94}; // in mm

    public static final int TICKS_PER_TILE = 40000;

    Telemetry telemetry;

    Drivebase drive;
    Gyro gyro;

    public Navigation(Drivebase drive, Gyro gyro, Telemetry telemetry){
        this.drive = drive;
        this.gyro = gyro;


        this.telemetry = telemetry;

        this.x = 0;
        this.y = 0;

        updatePosition();
    }

    public void updatePosition(){
        updateEncoders();

        this.gyroHeading = gyro.getHeading();
        this.deltaHeading = gyro.getDeltaHeading();

        // update x position
        this.x = 2 * (xEncoder / gyroHeading + 12757.38) * (Math.sin(gyroHeading / 2));
//        this.x = xEncoder;

        // update y position
        this.y = 2 * (yEncoder / gyroHeading + 4538.41) * (Math.sin(gyroHeading / 2));
//        this.y = yEncoder;

    }

    public int runToPosition(double x, double y, double heading, boolean xFirst, int cycle){
        boolean done;

        if (xFirst)
            done = runToPosition(x, 0, 0);
        else if (cycle == 0)
            done = runToPosition(0, 0, heading);
        else
            done = runToPosition(0, y, 0);


        if (done)
            return cycle--;
        else
            return cycle;
    }

    /**
     * @param x target x position
     * @param y target y position
     * @param heading target heading in degrees
     * @return whether or not the robot is at target position and facing target direction
     */
    public boolean runToPosition(double x, double y, double heading){
        updatePosition();

        double xDiff = x - this.x;
        double yDiff = this.y - y;
        double rotDiffCounterClock = (this.gyroHeading - (Math.toRadians(heading))) % (2 * Math.PI);
        double rotDiffClock = ((Math.toRadians(heading)) - this.gyroHeading) % (2 * Math.PI);

        double xPow = 0;
        double yPow = 0;
        double rotPow = 0;


        if (Math.abs(xDiff) > TICKS_PER_TILE / 25.0) {
            xPow = xDiff / 10.0;
        } else if (x == 0) {
            xPow = 0;
        }

        if (Math.abs(yDiff) > TICKS_PER_TILE / 25.0) {
            yPow = yDiff / 10.0;
        } else if (y == 0) {
            yPow = 0;
        }

        if (rotDiffClock >= rotDiffCounterClock){
            rotPow = rotDiffClock / 10.0;
        } else if (Math.abs(rotDiffCounterClock - rotDiffClock) > Math.toRadians(2)){
            rotPow = rotDiffCounterClock /10.0;
        } else if (heading == 0)
            rotPow = 0;

        drive.calculateDrivePowers(xPow , yPow, rotPow, true);
        telemetry.addLine("y = " + this.y);
        telemetry.addLine("x = " + this.x);
        telemetry.addLine("heading = " + this.gyroHeading);
        telemetry.addLine(String.valueOf(xPow + yPow + rotPow == 0));
        return xPow + yPow + rotPow < 0.1;

    }

    /**
     * Helper function that updates the encoder values every cycle
     */
    private void updateEncoders() {
        deltaX = xEncoder + drive.getEncoderTicks()[1];
        deltaY = yEncoder + drive.getEncoderTicks()[0];

        xEncoder = -drive.getEncoderTicks()[1];
        yEncoder = -drive.getEncoderTicks()[0];


    }

    /**
     * @return x coordinate
     */
    public double getX(){
        return x;
    }

    /**
     * @return y coordinate
     */
    public double getY(){
        return y;
    }

    /**
     * @return heading of the robot in only radians
     */
    public double getGyroHeading(){
        return gyroHeading;
    }
}