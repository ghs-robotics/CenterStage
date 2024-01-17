package org.firstinspires.ftc.teamcode.control.roadrunner;

import static org.firstinspires.ftc.teamcode.control.roadrunner.Localizer.*;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;

public class BallDriveKinematics{
    private double trackWidth;
    private double lateralMultiplier;

    DualNum[] ballIncrements;

    DualNum<Params> left;
    DualNum<Params> right;
    DualNum<Params> back;


    public BallDriveKinematics(double trackWidth, double wheelBase, double lateralMultiplier){
        this.trackWidth = (trackWidth + wheelBase) / 2;
        this.lateralMultiplier = lateralMultiplier;

        ballIncrements = new DualNum[]{left, right, back};
    }

    /**
     * @param motors use wheelIncrements from this class
     * @return adjusted powers
     */
    public Twist2dDual<Params> forward(DualNum<Params>[] motors){
        Twist2dDual<Params> powers = new Twist2dDual<Params>(
                new Vector2dDual<Params>(back, (left + right) * 0.5, );


    }

}
