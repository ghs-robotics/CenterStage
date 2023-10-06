package org.firstinspires.ftc.teamcode.bot.control;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;

public class Navigation {
    // axis based on the robot's starting position
    private double x;
    private double y;
    private double odoHeading;
    private double gyroHeading;

    private int leftEncoder;
    private int rightEncoder;
    private int backEncoder;

    // you would think balldrive odo would be easier to code
    // but the numbers are difficult to guess

    // 9/10 balldrive can't be precisely tuned, there is not enough traction between wheel and motor

    //139.5mm - left
    //139.5mm - right
    //108.54mm - back
    //
    //Diagonal vertical motors 163.7 mm
    //Diagonal horizontal motor 163.1 mm
    private final double leftTrackingDistance = 139.5 * 3;
    private final double rightTrackingDistance = 139.5 * 3;
    private final double backTrackingDistance = 108.54 * 3;

    Telemetry telemetry;

    Drivebase drive;
    Gyro gyro;

    public Navigation(Drivebase drive, Gyro gyro, Telemetry telemetry){
        this.drive = drive;
        this.gyro = gyro;

        this.telemetry = telemetry;

        this.x = 0;
        this.y = 0;

        gyro.resetHeading();
//        updatePosition();
    }

    public void updatePosition(){
        updateEncoders();

        // update rotation using encoders in rads
        this.odoHeading = (leftEncoder - rightEncoder) / (leftTrackingDistance + rightTrackingDistance);

        this.gyroHeading = gyro.getHeading(AngleUnit.RADIANS);

        // update x position
        this.x = 2 * (backEncoder / gyroHeading + backTrackingDistance) * (Math.sin(gyroHeading / 2));

        // update y position
        this.y = -2 * (leftEncoder / gyroHeading + leftTrackingDistance) * (Math.sin(gyroHeading / 2));

    }

    public boolean runToPosition(double x, double y, double heading){
        updatePosition();

        double xDiff = this.x - x;
        double yDiff = this.y - y;

        double xPow = 0;
        double yPow = 0;
        double rotPow = 0;


        if (Math.abs(xDiff) > 10) {
            xPow = xDiff / 100;
        }
        if (Math.abs(yDiff) > 10) {
            yPow = yDiff / 100;
        }

        // todo note-
        //  the power is not going to 0 at the end

        drive.calculateDrivePowers(xPow, yPow, rotPow);
        telemetry.addLine("x = " + x);
        telemetry.addLine("y = " + y);
        return !(Math.abs(xDiff) > 10) && !(Math.abs(yDiff) > 10);
    }

    private void updateEncoders() {
        leftEncoder = drive.getEncoderTicks()[0];
        rightEncoder = drive.getEncoderTicks()[1];
        backEncoder = drive.getEncoderTicks()[2];
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    /**
     * @return heading of the robot in only radians
     */
    public double getOdoHeading(){
        return odoHeading;
    }

    public double getGyroHeading(){
        return gyroHeading;
    }

}
