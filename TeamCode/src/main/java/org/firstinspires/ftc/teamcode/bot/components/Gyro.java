package org.firstinspires.ftc.teamcode.bot.components;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Gyro {
    IMU gyro;
    Orientation orientation;
    RevHubOrientationOnRobot revOrientation;
    YawPitchRollAngles angles;

    double yaw, pitch, roll;
    double lastYaw;

    double headingVel;
    double lastHeadingVel;

    public Gyro(HardwareMap hardwareMap) {
        gyro = hardwareMap.get(IMU.class, "imu");

        orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.YXZ, DEGREES,
                0, 0, 0, 0);

        revOrientation = new RevHubOrientationOnRobot(orientation);

        IMU.Parameters parameters = new IMU.Parameters(revOrientation);

        gyro.initialize(parameters);

    }

    public double getDeltaHeading(){
        return yaw - lastYaw;
    }

    public double getHeading(){
        lastYaw = yaw;
        yaw = getOrientation(RADIANS)[0];
        return yaw;
    }

    public double getHeadingVelocity(){
        headingVel = gyro.getRobotAngularVelocity(RADIANS).zRotationRate;
        double headingVelOffest = 0;

        if (Math.abs(headingVel - lastHeadingVel) > Math.PI)
            headingVelOffest -= Math.signum(headingVel) * 2 * Math.PI;
        lastHeadingVel = headingVel;

        return headingVelOffest + headingVel;
    }

    // up and down x axis
    public double getPitch(AngleUnit unit){
        pitch = getOrientation(unit)[1];
        return pitch;
    }

    // up and down y axis
    public double getRoll(AngleUnit unit){
        roll = getOrientation(unit)[2];
        return roll;
    }

    public void resetHeading() {
        gyro.resetYaw();
    }

    private double[] getOrientation(AngleUnit unit) {
        angles = gyro.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw(unit);
        double pitch = angles.getPitch(unit);
        double roll = angles.getRoll(unit);

        double[] angle = {yaw, pitch, roll};

        return angle;
    }
}