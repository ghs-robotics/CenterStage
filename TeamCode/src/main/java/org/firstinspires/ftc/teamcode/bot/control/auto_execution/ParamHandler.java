package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import java.sql.ParameterMetaData;

public class ParamHandler {
    int x;
    int y;
    double heading;
    int intakeLevel;

    public ParamHandler(){

    }

    public ParamHandler(int intakeLevel){
        this.intakeLevel = intakeLevel;
    }

    public ParamHandler(int action, int x, int y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
