package org.firstinspires.ftc.teamcode.control.roadrunner;

public class BallDriveKinematics {
    private double trackWidth;
    private double lateralMultiplier;

    public BallDriveKinematics(double trackWidth, double wheelBase, double lateralMultiplier){
        this.trackWidth = (trackWidth + wheelBase) / 2;
        this.lateralMultiplier = lateralMultiplier;
    }

//    public class WheelIncrements<Param>
}
