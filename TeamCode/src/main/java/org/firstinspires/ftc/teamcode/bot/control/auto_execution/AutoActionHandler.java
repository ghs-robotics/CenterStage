package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;

import java.util.ArrayList;

public class AutoActionHandler {
    private ArrayList<AutoActions> actionList;
    private AutoActions current;

    private Robot robot;
    private Telemetry telemetry;

    private int totalActions;
    private int zone;


    public AutoActionHandler(Robot robot, Telemetry telemetry){
        this.actionList = new ArrayList<AutoActions>();
        this.robot = robot;
        this.telemetry = telemetry;
    }

    /**
     * runs the action and calls next action in case the current action is complete.
     */
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

    /**
     * Checks the status of the current action and removes the action from queue if isFinished
     * returns true.
     */
    private void nextAction(){
        if (current.isFinished()) {
            current = null;
            actionList.remove(0);
            current = actionList.get(0);
        }
    }

    /**
     * Gets the zone (spike mark) that was detected by the camera.
     */
    public void findAndSetZone(){
        zone = robot.cam.getZone();
        for (AutoActions a: actionList)
            a.setZone(zone);
    }

    /**
     * @return The action list of this object.
     *
     * Made for getting presets and adding them to the main Auto queue
     */
    public ArrayList<AutoActions> getActions(){
        return actionList;
    }

    /**
     * starts the queue
     */
    public void init(){
        if (actionList.isEmpty())
            return;

        current = actionList.get(0);
        totalActions = actionList.size();
        actionList.add(new AutoActions(AutoActions.DONE, robot));
    }

    /**
     * @return the total number of actions queue to execute
     */
    public int getTotalActions(){
        return totalActions;
    }

    /**
     * Prints the current step in the Auto and gives an idea of how complete the auto is.
     */
    public void status(){
        int currentStep = totalActions - actionList.size() + 1;

        if (current.getIdentity() != AutoActions.DONE) {
            telemetry.addLine(current.getDescription());
            telemetry.addLine(currentStep + " of " + totalActions + " actions");
        }else
            telemetry.addLine( "Done!");
    }

}
