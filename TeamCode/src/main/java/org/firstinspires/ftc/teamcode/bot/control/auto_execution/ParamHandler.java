package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.INTAKE;

public class ParamHandler {
    int x;
    int y;
    double heading; // in degrees
    boolean driveXFirst;

    int intakeLevel;
    int liftLevel;
    int outtakeLevel;

    int waitTime;

    public ParamHandler(){

    }

    public ParamHandler(int seconds){
        waitTime = seconds;
    }

    public ParamHandler(int id, int height){
        if (id == INTAKE)
            this.intakeLevel = height;
        if (id == LIFT)
            this.liftLevel = height + 100;
    }

    public ParamHandler (int id, int lift, int outtake){
        if (id == LIFT){
            this.liftLevel = lift;
            this.outtakeLevel = outtake;
        }
    }


    public ParamHandler (int x, int y, double heading, boolean xFirst){
        this(x, y, heading);
        driveXFirst = xFirst;
    }

    public ParamHandler (double[] pos){
        this.x = (int) pos[0];
        this.y = (int) pos[1];
        this.heading = pos[2];
    }

    public ParamHandler( int x, int y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
