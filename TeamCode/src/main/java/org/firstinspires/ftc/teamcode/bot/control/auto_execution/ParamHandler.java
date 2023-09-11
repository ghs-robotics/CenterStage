package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

public class ParamHandler {
    private boolean hasParams;
    private double[] targetPosition = new double[3];


    public ParamHandler(int action){
        hasParams = false;
    }

    public ParamHandler(int action, int x, int y, double heading){
        hasParams = true;

        targetPosition[0] = x;
        targetPosition[1] = y;
        targetPosition[2] = heading;
    }

    public boolean isHasParams(){
        return hasParams;
    }
}
