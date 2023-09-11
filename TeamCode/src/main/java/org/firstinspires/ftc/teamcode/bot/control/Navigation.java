package org.firstinspires.ftc.teamcode.bot.control;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;

public class Navigation {
    private double x;
    private double y;
    private double heading;

    private int leftEncoder;
    private int rightEncoder;
    private int backEncoder;

    private final double leftTrackingDistance = 4;
    private final double rightTrackingDistance = 4;
    private final double backTrackingDistance = 4;

    Drivebase drive;
    Gyro gyro;

    public Navigation(Drivebase drive, Gyro gyro){
        this.drive = drive;
        this.gyro = gyro;

        updatePosition();
    }

    public void updatePosition(){
        updateEncoders();

        // update x position
        this.x = 2 * (backEncoder / heading + backTrackingDistance) * (Math.sin(heading / 2));

        // update y position
        this.y = 2 * (rightEncoder / heading + rightTrackingDistance) * (Math.sin(heading / 2));

        // update rotation using encoders in rads
        this.heading = (leftEncoder - rightEncoder) / (leftTrackingDistance + rightTrackingDistance);

        this.heading = gyro.getHeading(AngleUnit.RADIANS);

    }

    private void updateEncoders() {
        leftEncoder = (int) Math.round(drive.getEncoderTicks()[0]);
        rightEncoder = (int) Math.round(drive.getEncoderTicks()[1]);
        backEncoder = (int) Math.round(drive.getEncoderTicks()[2]);
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
    public double getHeading(){
        return heading;
    }

}
