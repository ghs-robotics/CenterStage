package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.Robot;

public class AutoActions {
    // identities
    public static final int DONE = -1;
    public static final int MOVE = 0;
    public static final int INTAKE = 1;
    public static final int DELIVER = 2;
    public static final int PLACE = 3;
    public static final int ALIGN = 4;

    private String description;

    private int identity;
    private boolean endAction;

    private ElapsedTime timer;
    private boolean timerReset;

    private int zone;

    ParamHandler params;
    private Robot robot;

    public AutoActions(int id, Robot robot){
        this.identity = id;
        this.robot = robot;
        timerReset = false;
        timer = new ElapsedTime();
//        setDescription();
    }

    public AutoActions(int id, Robot robot,ParamHandler params){
        this(id, robot);
        this.params = params;
    }

    private void moveTo(){
        endAction = robot.nav.runToPosition(params.x, params.y, params.heading);
    }

    private void runIntake(){
//        robot.intake.setIntakeHeight(params.intakeLevel);
//
//        if(!timerReset)
//            timer.reset();
//
//        endAction = robot.intake.autoRunIntake(timer.milliseconds());
    }

    private void runDelivery(){
        // same as intake
    }

    private void placePixel(){
        // drops the pixel
    }

    private void alignBotToTag(){
        // looks for the required tag
        // requires the use of moving to align itself
        // runs the delivery

    }

    /**
     * @return whether or not this action has been completed
     */
    public boolean isFinished(){
        return endAction;
    }

    /**
     * determines the action and what this specific action will do.
     */
    public void runAction(){
        switch (identity){
            case MOVE:
                moveTo();
                break;
            case INTAKE:
                runIntake();
                break;
            case DELIVER:
                runDelivery();
                break;
            case PLACE:
                placePixel();
                break;
            case ALIGN:
                alignBotToTag();
                break;

        }
    }

    /**
     * @return description of the specific object's action and status
     */
    public String getDescription(){
        return description;
    }

    /**
     * helper method to get telemetry text
     */
    private void setDescription() {
        switch (identity){
            case MOVE:
                description = "Moving " + params.x + ", " + params.y + ". Facing " + params.heading;
                break;
            case INTAKE:
                description = "Running Intake for " + (timer.milliseconds() / 1000.0) + " sec";
                break;
            case DELIVER:
                description = "Delivering pixel to backdrop";
                break;
            case PLACE:
                description = "Placing pixel on spike mark";
                break;
            case ALIGN:
                description = "Aligning with the AprilTag";
                break;
        }
    }

    /**
     * sets the zone read by the camera
     */
    public void setZone(int zone){
        this.zone = zone;
    }

    /**
     * @return the identity of this action
     */
    public int getIdentity(){
        return identity;
    }
}
