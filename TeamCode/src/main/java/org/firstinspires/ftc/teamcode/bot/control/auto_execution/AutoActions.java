package org.firstinspires.ftc.teamcode.bot.control.auto_execution;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.NavigationPID;

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
    public static final int DROP = 6; // drops it out of the outtake
    public static final int EXTEND = 7;
    public static final int RETRACT = 8;
    public static final int DETECT = 9;


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
    int liftLevel;
    int outtakeLevel;

    double waitTime;

    private NavigationPID xPID;
    private NavigationPID yPID;
    private NavigationPID liftPID;

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

        double outPutLimit = 2;
        double integralLimit = 3650;

        xPID = new NavigationPID(.152, .00162824, .001674);
        yPID = new NavigationPID(.152, .00162824, .001674);

        xPID.setOutputLimits(outPutLimit);
        yPID.setOutputLimits(outPutLimit);

        xPID.setMaxIOutput(integralLimit);
        yPID.setMaxIOutput(integralLimit);

        if (!robot.RED)
            this.x *= -1;

        xPID.setTarget(x);
        yPID.setTarget(y);

    }
    public AutoActions(int id, Robot robot, double value){
        this(id, robot);
        if (id == INTAKE)
            this.intakeLevel = (int) value;
        if (id == LIFT) {
            this.liftLevel = (int) (value + 100);
            double outPutLimit = 2;
            double integralLimit = 2500;

            //todo tune values here - run test auto
            liftPID = new NavigationPID(.152, .00162824, .001674);
            liftPID.setOutputLimits(outPutLimit);
            liftPID.setMaxIOutput(integralLimit);
            liftPID.setTarget(liftLevel);

        }if (id == WAIT)
            waitTime = value;
    }
    public AutoActions (int id, Robot robot, double[] pos){
        this(id, robot, (int) pos[0], (int) pos[1], (int) pos[2]);
    }
    /**
     * Driving the rob
     */
    private void moveTo(){
        resetTimer();

//        boolean there = robot.nav.runToPosition(x, y, heading);
        boolean there = robot.drive.runToPosition(xPID, yPID);
        endAction = there;//||  timer.milliseconds() > 5000;
    }

//    private void dropPixels(){
//        resetTimer();
//
//        robot.delivery.autoDropPixels(Delivery.DROPPER_SECOND);
//        if (timer.milliseconds() > 700)
//            robot.delivery.autoDropPixels(Delivery.DROPPER_INTAKING);
//
//        endAction = timer.milliseconds() > 1000;
//    }

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
        robot.delivery.driveLiftToPosition(liftPID);
        resetTimer();
        endAction = timer.milliseconds() > 750;
    }

    private void extendDropper(){
        resetTimer();
        if(timer.milliseconds() < 1400)
//            robot.delivery.setExtendPower(-1);
//        else
//            robot.delivery.setExtendPower(0);
                endAction = timer.milliseconds() > 2000;
    }

    private void retractDropper(){
        resetTimer();

//        endAction = robot.delivery.autoRunExtension(1, timer.milliseconds());
    }

    /**
     * drops pixel by reversing Intake
     */
    private void placePixel(){
        robot.intake.autoPixelOut();
        resetTimer();
        if(timer.milliseconds() > 2500)
            endAction = true;
    }

    private void detectSpikeMark(){
        if (!timerReset)
            robot.cam.detectProp();
        resetTimer();
        endAction = timer.milliseconds() > 750;
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
//            case DROP:
//                dropPixels();
//                break;
            case EXTEND:
                extendDropper();
                break;
            case RETRACT:
                retractDropper();
                break;
            case DETECT:
                detectSpikeMark();
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

    public int getTimer() {
        return (int) (timer.milliseconds() / 1000);
    }
}
