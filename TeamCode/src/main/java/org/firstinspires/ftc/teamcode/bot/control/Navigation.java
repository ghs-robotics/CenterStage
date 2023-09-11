package org.firstinspires.ftc.teamcode.bot.control;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;

public class Navigation {
    private int x;
    private int y;
    private double heading;

    Drivebase drive;
    Gyro gyro;

    public Navigation(Drivebase drive, Gyro gyro){
        this.drive = drive;
        this.gyro = gyro;

        updatePosition();
    }

    public void updatePosition(){
        this.x = (int) Math.round(drive.getEncoderTicks()[0]);
        this.y = (int) Math.round(drive.getEncoderTicks()[1]);
        this.heading = gyro.getHeading(AngleUnit.DEGREES);

    }

    public int getX(){
        return x;
    }

    public int getY(){
        return y;
    }

    public double getHeading(){
        return heading;
    }

}
