package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import org.firstinspires.ftc.teamcode.bot.Robot;

import java.util.ArrayList;

public class AutoRunner {
    private ArrayList<AutoActions> actionList;
    private AutoActions current;

    private Robot robot;

    private int zone;

    public AutoRunner(Robot robot){
        this.actionList = new ArrayList<AutoActions>();
        this.robot = robot;
    }

    public void run(){
        current.runAction();

            zone = current.getZone();
        nextAction();
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

    public int init(){
        current = actionList.get(0);
        return zone;
    }
}
