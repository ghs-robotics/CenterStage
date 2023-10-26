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

    private int verticalEncoder;
    private int horizontalEncoder;

    // you would think balldrive odo would be easier to code
    // but the numbers are difficult to guess

    // 9/10 balldrive can't be precisely tuned, there is not enough traction between wheel and motor

    //139.5mm - left
    //139.5mm - right
    //108.54mm - back
    //
    //Diagonal vertical motors 163.7 mm
    //Diagonal horizontal motor 163.1 mm
    private final double verticalTrackingDistance = 139.5 * 2;
    private final double horizontalTrackingDistance = 108.54 * 2;

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
        this.x = Math.cos(gyroHeading) * horizontalEncoder;

        // update y position
        this.y = Math.sin(gyroHeading) * verticalEncoder;

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

        drive.calculateDrivePowers(xPow , yPow, rotPow);
        telemetry.addLine("y = " + this.y);
        telemetry.addLine("x = " + this.x);
        telemetry.addLine("heading = " + this.gyroHeading);
        return xPow + yPow + rotPow == 0;
    }

    /**
     * Helper function that updates the encoder values every cycle
     */
    private void updateEncoders() {
        verticalEncoder = drive.getEncoderTicks()[0];
        horizontalEncoder = drive.getEncoderTicks()[1];
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