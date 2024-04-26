package org.firstinspires.ftc.teamcode.control.auto_execution;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.ERROR;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.WAIT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;

import java.util.ArrayList;

public class AutoActionHandler {
    private final ArrayList<AutoActions> actionList;
    private AutoActions current;

    private final Robot robot;
    private final Telemetry telemetry;

    private int totalActions;

    private boolean farStart;

    public AutoActionHandler(Robot robot, Telemetry telemetry){
        this.actionList = new ArrayList<AutoActions>();
        this.robot = robot;
        this.telemetry = telemetry;
    }


    public AutoActionHandler(Robot robot, Telemetry telemetry, boolean farStart){
        this.actionList = new ArrayList<AutoActions>();
        this.robot = robot;
        this.farStart = farStart;
        this.telemetry = telemetry;
    }


    /**
     * runs the action and calls next action in case the current action is complete.
     */
    public void run(){
        current.runAction();
        nextAction();
        robot.update();
    }

    /**
     * @param actionSet a pre-existing set of autoActions to add to this list
     */
    public void add(ArrayList<AutoActions> actionSet){
        actionList.addAll(actionSet);
    }

    /**
     * @param actionHandler gets a pre-existing set of actions to add to this list from a pre-built
     *                      AutoActionHandler
     */
    public void add(AutoActionHandler actionHandler){
        actionList.addAll(actionHandler.getActions());
    }

    /**
     * Built for moving
     * @param action id of the action but for this specific function, most likely MOVE
     * @param x target x position
     * @param y target y position
     * @param heading target heading (usually 0)
     */
    public void add (int action, int x, int y, double heading){
        if (x == 0 && y == 0)
            return;
        actionList.add(new AutoActions(action, robot, x, y, heading));
        add(WAIT, .1);
    }

    /**
     * Built for moving
     * @param action id of the action but for this specific function, most likely MOVE
     * @param pos target position array
     */
    public void add (int action, double[] pos){
        actionList.add(new AutoActions(action, robot, pos));
        add(WAIT, .1);
    }

    /**
     * Built for moving, used if you want to make the bot drive along x and then y
     * @param action id of the action but for this specific function, most likely MOVE
     * @param x target x position
     * @param y target y position
     * @param heading target heading (usually 0)
     * @param split whether or not you want to split the move function to move x and then y
     */
    public void add (int action, int x, int y, double heading, boolean split){
        if (split && x != 0 || y != 0){
            add(action, x, 0, heading);
            add(action, 0, y, heading);
        }else
            add(action, x, y, heading);
    }

    /**
     * Built for moving, used if you want to make the bot drive along x and then y
     * @param action id of the action
     * @param pos target position array
     * @param split whether or not you want to split the move function to move x and then y
     */
    public void add (int action, double[] pos, boolean split){
        add(action, (int) pos[0], (int) pos[1], pos[2], split);

    }

    /**
     * Standard add function with one parameter
     *
     * @param action the id of the action queued
     * @param value any parameters the action requires
     */
    public void add(int action, double value) {
        actionList.add(new AutoActions(action, robot, value));
    }

    /**
     * The basic add function for any actions that don't require parameters
     *
     * @param action the identity of the action (see the public static constant in AutoActions)
     *               This one is for actions that do not require parameters
     */
    public void add(int action){
        if (action == DELIVER){
            add(LIFT);
            add(EXTEND);
            add(DROP);
            add(RETRACT);
        } else {
            actionList.add(new AutoActions(action, robot));
        }
    }
    /**
     * Checks the status of the current action and removes the action from queue if isFinished
     * returns true.
     */
    private void nextAction(){
        if (current.isEmergencyStop()){
            actionList.clear();
            add(ERROR);

            telemetry.addLine("EMERGENCY STOP");
        }
        if (current.isFinished()) {
            current = null;
            actionList.remove(0);
            current = actionList.get(0);
        }
    }

    /**
     * @return The action list of this object.
     *
     * Made for getting presets and adding them to the main AutoRed queue
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

        for (AutoActions a: actionList){
            a.setFarStart(farStart);
        }

        actionList.add(new AutoActions(AutoActions.DONE, robot));
        current = actionList.get(0);
        totalActions = actionList.size();
    }

    /**
     * @return the total number of actions queue to execute
     */
    public int getTotalActions(){
        return actionList.size();
    }

    /**
     * Prints the current step in the AutoRed and gives an idea of how complete the auto is.
     */
    public void status(){
        int currentStep = totalActions - actionList.size() + 1;

        telemetry.addLine(currentStep + " of " + totalActions + " actions");
        telemetry.addLine();

        if (current.getIdentity() == AutoActions.ERROR) {
            telemetry.addLine("Emergency stop initiated. Bad wheels :c");
        }else if(current.getIdentity() != AutoActions.DONE)
            telemetry.addLine(current.getDescription());
        else {
            telemetry.addLine("Done!");
        }

        telemetry.addLine();
        robot.getAutoTelemetry();
    }

    public void update(){
        run();
        status();
    }

}
