package org.firstinspires.ftc.teamcode.control.roadrunner;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.TankKinematics;

public class BallDriveKinematics{
    private double trackWidth;
    private double lateralMultiplier;

    DualNum<Localizer.Params>[] wheelIncrements;

    DualNum<Localizer.Params> left;
    DualNum<Localizer.Params> right;
    DualNum<Localizer.Params> back;


    public BallDriveKinematics(double trackWidth, double wheelBase, double lateralMultiplier){
        this.trackWidth = (trackWidth + wheelBase) / 2;
        this.lateralMultiplier = lateralMultiplier;

        wheelIncrements = new DualNum[]{left, right, back};
    }

    public

}
