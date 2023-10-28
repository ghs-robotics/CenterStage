package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.INTAKE;

import java.sql.ParameterMetaData;

public class ParamHandler {
    int x;
    int y;
    double heading; // in degrees
    boolean driveXFirst;

    int intakeLevel;
    int outtakeLevel;

    int waitTime;

    public ParamHandler(){

    }

    public ParamHandler(int seconds){
        waitTime = seconds;
    }

    public ParamHandler(int id, int height){
        if (id == INTAKE)
            this.intakeLevel = intakeLevel;
        else if (id == DELIVER)
            outtakeLevel = height;
    }

    public ParamHandler (int x, int y, double heading, boolean xFirst){
        this(x, y, heading);
        driveXFirst = xFirst;
    }

    public ParamHandler( int x, int y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
