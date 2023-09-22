package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;

import java.util.ArrayList;

public class AutoActionHandler {
    private ArrayList<AutoActions> actionList;
    private AutoActions current;

    private Robot robot;
    private Telemetry telemetry;

    private int totalSteps;
    private int zone;

    public AutoActionHandler(Robot robot, Telemetry telemetry){
        this.actionList = new ArrayList<AutoActions>();
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run(){
        current.runAction();
        nextAction();
    }

    public void add(ArrayList<AutoActions> actionSet){
        actionList.addAll(actionSet);
    }

    public void add(AutoActionHandler actionSet){
        actionList.addAll(actionSet.getActions());
    }

    public void add(int action, ParamHandler params){
        actionList.add(new AutoActions(action, robot, params));
    }

    public void add(int action){
        actionList.add(new AutoActions(action, robot));
    }

    private void nextAction(){
        if (current.isFinished()) {
            actionList.remove(0);
            current = actionList.get(0);
        }
    }

    public void findAndSetZone(){
        //todo get the zone from the camera
        zone = robot.cam.getZone();
        for (AutoActions a: actionList)
            a.setZone(zone);
    }

    public ArrayList<AutoActions> getActions(){
        return actionList;
    }

    public void init(){
        current = actionList.get(0);
        totalSteps = actionList.size();
    }

    public void status(){
        int currentStep = totalSteps - actionList.size();

        if (!actionList.isEmpty()) {
            telemetry.addLine(current.getDescription());
            telemetry.addLine(currentStep + " of " + totalSteps + " steps");
        }else
            telemetry.addLine( "Done!;");
    }

}
