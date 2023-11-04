package org.firstinspires.ftc.teamcode.bot.control;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;

public class Navigation {
    // axis based on the robot's starting position
    private double x;
    private double y;
    private double gyroHeading;

    private int yEncoder;
    private int xEncoder;

    private final int TICKS_PER_REV = 8192;
    private final double WHEEL_RAD = 17.5; // in mm

    private final double[] X_DIS_FROM_CENTER = new double[]{164.109, 48.88}; // in mm
    private final double[] Y_DIS_FROM_CENTER = new double[]{153.275, 60.916}; // in mm

    public final double TICKS_PER_TILE = 39800;

    Telemetry telemetry;

    Drivebase drive;
    Gyro gyro;

    public Navigation(Drivebase drive, Gyro gyro, Telemetry telemetry){
        this.drive = drive;
        this.gyro = gyro;

        this.telemetry = telemetry;

        this.x = 0;
        this.y = 0;

//        updatePosition();
    }

    public void updatePosition(){
        updateEncoders();

        this.gyroHeading = gyro.getHeading(AngleUnit.RADIANS);

        // update x position
//        this.x = Math.sin(gyroHeading) * xEncoder;
        this.x = xEncoder;

        // update y position
//        this.y = Math.cos(gyroHeading) * yEncoder;
        this.y = yEncoder;

    }
//
//    public boolean runToPosition(double x, double y, double heading, boolean xFirst){
//        return runToPosition(x, y, heading, xFirst, 1);
//    }
//
//    private boolean runToPosition(double x, double y, double heading, boolean xFirst, int cycle){
//        if (cycle < 1)
//            return true;
//
//        if (xFirst)
//            runToPosition(x, this.y, heading);
//        else
//            runToPosition(this.x, y, heading);
//
//        return runToPosition(x, y, heading, !xFirst, cycle--);
//    }

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


        if (Math.abs(xDiff) > 15) {
            xPow = xDiff / 10.0;
        }

        if (Math.abs(yDiff) > 15) {
            yPow = yDiff / 10.0;
        }

        if (rotDiffClock >= rotDiffCounterClock){
            rotPow = rotDiffClock / 10.0;
        } else if (Math.abs(rotDiffCounterClock - rotDiffClock) > Math.toRadians(2)){
            rotPow = rotDiffCounterClock /10.0;
        }

        drive.calculateDrivePowers(xPow , yPow, rotPow, true);
        telemetry.addLine("y = " + this.y);
        telemetry.addLine("x = " + this.x);
        telemetry.addLine("heading = " + this.gyroHeading);
        return xPow + yPow + rotPow == 0;
    }

    /**
     * Helper function that updates the encoder values every cycle
     */
    private void updateEncoders() {
        yEncoder = -drive.getEncoderTicks()[0];
        xEncoder = -drive.getEncoderTicks()[1];
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