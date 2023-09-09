package org.firstinspires.ftc.teamcode.robot.control;

import org.firstinspires.ftc.teamcode.robot.components.Gyro;
import org.firstinspires.ftc.teamcode.robot.components.drive.Drivebase;

public class Navigation {
    private int x;
    private int y;
    private double heading;

    Drivebase drive;
    Gyro gyro;

    public Navigation(Drivebase drive, Gyro gyro){
        this.drive = drive;
        this.gyro = gyro;
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
