package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets.centerBackDropPos;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets.centerSpikePos;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets.leftBackDropPos;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets.leftSpikePos;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets.rightBackDropPos;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets.rightSpikePos;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;
import org.firstinspires.ftc.teamcode.bot.control.PID;

public class AutoActions {
    // identities
    public static final int DELIVER = -2; // not used here, used in Handler to add a series of actions
    public static final int DONE = -1;
    public static final int MOVE = 0;
    public static final int INTAKE = 1;
    public static final int LIFT = 2;
    public static final int PLACE_PIXEL = 3; // spits it out the intake
    public static final int ALIGN = 4;
    public static final int WAIT = 5;
    public static final int DROP = 6; // drops it out of the
    public static final int EXTEND = 7;
    public static final int RETRACT = 8;
    public static final int DETECT = 9;
    public static final int MOVE_TO_SPIKE = 10;
    public static final int MOVE_TO_BACKDROP = 11;


    private Robot robot;

    private int identity;
    private boolean endAction;

    private ElapsedTime timer;
    private boolean timerReset;

    private int zone;
    private String description;

    double x;
    double y;
    double heading; // in degrees

    int intakeLevel;
    int liftLevel = 600;
    int outtakeLevel;

    double waitTime;

    private PID xPID;
    private PID yPID;

    public AutoActions(int id, Robot robot){
        this.identity = id;
        this.robot = robot;
        timerReset = false;
        timer = new ElapsedTime();


        setDescription();
    }

    public AutoActions(int id, Robot r, int x, int y, double heading){
        this(id, r);
        this.x = x;
        this.y = y;
        this.heading = heading;

        setNavPID();

        checkXSign();

        xPID.setTarget(x);
        yPID.setTarget(y);

    }
    public AutoActions(int id, Robot robot, double value){
        this(id, robot);
        if (id == INTAKE)
            // target level should be 600
            this.intakeLevel = (int) value;
        else if (id == LIFT)
            this.liftLevel = (int) (value);
        else if (id == WAIT)
            waitTime = value;
        else if (id == MOVE_TO_SPIKE) {
            zone = (int) value;
        }else if (id == MOVE_TO_BACKDROP) {
            zone = (int) value;
        }
    }
    public AutoActions (int id, Robot robot, double[] pos){
        this(id, robot, (int) pos[0], (int) pos[1], (int) pos[2]);
    }
    /**
     * Driving the rob
     */
    private void moveTo(){
        resetTimer();

        boolean there = robot.drive.runToPosition(xPID, yPID);
        boolean timeOut = timer.milliseconds() > 7000;
        endAction = there || timeOut;
    }

    private void moveToSpike(){
        setNavPID();
        if (zone == 1){
            x = leftSpikePos[0];
            yPID.setTarget(leftSpikePos[1]);
        } else if (zone == 2) {
            x = (centerSpikePos[0]);
            yPID.setTarget(centerSpikePos[1]);
        } else {
            x = (rightSpikePos[0]);
            yPID.setTarget(rightSpikePos[1]);
        }

        checkXSign();

        xPID.setTarget(x);

        identity = MOVE;
    }

    private void moveToBackdrop(){
        setNavPID();
        if (zone == 1){
            x = (leftBackDropPos[0]);
            yPID.setTarget(leftBackDropPos[1]);
        } else if (zone == 2) {
            x = (centerBackDropPos[0]);
            yPID.setTarget(centerBackDropPos[1]);
        } else {
            x = (rightBackDropPos[0]);
            yPID.setTarget(rightBackDropPos[1]);
        }

        checkXSign();

        xPID.setTarget(x);

        identity = MOVE;
    }

    private void dropPixels(){
        resetTimer();

        robot.delivery.autoDropPixels(Delivery.DROPPER_SECOND);
        if (timer.milliseconds() > 700)
            robot.delivery.autoDropPixels(Delivery.DROPPER_INTAKING);

        endAction = timer.milliseconds() > 400;
    }

    private void runIntake(){
        robot.intake.setIntakeHeight(intakeLevel);

        resetTimer();

        if (timer.milliseconds() < 2550)
            robot.intake.pixelIn(1);
        else
            robot.intake.pixelIn(0);
        endAction = timer.milliseconds() > 2700;
    }

    /**
     * Runs the lift
     */

    private void runLift(){
        // same as intake
        robot.delivery.driveLiftToPosition(300);
        resetTimer();
        endAction = timer.milliseconds() > 750;
    }

    private void extendDropper(){
        resetTimer();
        if(timer.milliseconds() < 1400)
            robot.delivery.setExtendPower(-1);
        else
            robot.delivery.setExtendPower(0);
        endAction = timer.milliseconds() > 2000;
    }

    private void retractDropper(){
        resetTimer();

        endAction = robot.delivery.autoRunExtension(1, timer.milliseconds());
    }

    /**
     * drops pixel by reversing Intake
     */
    private void placePixel(){
        robot.intake.setIntakeHeight(2);
        robot.intake.autoPixelOut();

        resetTimer();
        if(timer.milliseconds() > 1500) {
            endAction = true;
            robot.intake.pixelIn(0);
        }
    }

    private void detectSpikeMark(){
        if (!timerReset)
            robot.cam.detectProp();
        resetTimer();
        endAction = timer.milliseconds() > 1000;
    }

    private void alignBotToTag(){
        // looks for the required tag
        // requires the use of moving to align itself
        // runs the delivery
    }

    /**
     * waits out timer until timer is greater than or equal to the parameter wait time
     */
    private void waiting() {
        resetTimer();

        endAction = timer.milliseconds() > (waitTime * 1000);
    }

    private void shutOffBot(){
        robot.shutOff();
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
            case DONE:
                shutOffBot();
                break;
            case MOVE:
                moveTo();
                break;
            case INTAKE:
                runIntake();
                break;
            case LIFT:
                runLift();
                break;
            case PLACE_PIXEL:
                placePixel(); //spit it out the intake
                break;
            case ALIGN:
                alignBotToTag();
                break;
            case WAIT:
                waiting();
                break;
            case DROP:
                dropPixels();
                break;
            case EXTEND:
                extendDropper();
                break;
            case RETRACT:
                retractDropper();
                break;
            case DETECT:
                detectSpikeMark();
                break;
            case MOVE_TO_SPIKE:
                moveToSpike();
                break;
            case MOVE_TO_BACKDROP:
                moveToBackdrop();
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
        description = "";
        switch (identity){
            case MOVE:
            case MOVE_TO_SPIKE:
            case MOVE_TO_BACKDROP:
                description = "Driving";
                break;
            case INTAKE:
                description = "Running Intake for " + (timer.milliseconds() / 1000.0) + " sec";
                break;
            case LIFT:
                description = "Delivering pixel to backdrop";
                break;
            case PLACE_PIXEL:
                description = "Placing pixel on spike mark";
                break;
            case ALIGN:
                description = "Aligning with the AprilTag";
                break;
            case WAIT:
                description = "waiting";
                break;
            case EXTEND:
                description = "Extending the Outtake";
                break;
            case RETRACT:
                description = "retracting the Outtake";
                break;
        }
    }

    private void setNavPID(){
        double outPutLimit = 2;
        double integralLimit = 3650;

        xPID = new PID(.152, .00165964, .001664);
        yPID = new PID(.152, .00165964, .001664);

        xPID.setOutputLimits(outPutLimit);
        yPID.setOutputLimits(outPutLimit);

        xPID.setMaxIOutput(integralLimit);
        yPID.setMaxIOutput(integralLimit);}


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

    private void resetTimer(){
        if (!timerReset){
            timer.reset();
            timerReset = true;
        }
    }

    private void checkXSign(){
        if (!robot.RED)
            this.x *= -1;

    }

    public int getTimer() {
        return (int) (timer.milliseconds() / 1000);
    }
}
