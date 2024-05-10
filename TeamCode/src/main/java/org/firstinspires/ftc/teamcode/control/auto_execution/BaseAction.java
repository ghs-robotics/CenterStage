package org.firstinspires.ftc.teamcode.control.auto_execution;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.cv.Camera.SPIKE_ZONE;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.BACK_ADJUST_Y;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.BLUE_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.BLUE_SPIKE;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.RED_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.RED_SPIKE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.PIDControllers.NavigationPID;

public class BaseAction {

    // All the protected variables and functions in this class can be accessed in the AutoActions
    // class.
    protected int identity;
    protected boolean endAction;

    protected ElapsedTime timer;
    protected boolean timerReset;

    protected String description;

    protected boolean farStart;

    // properties for MOVE
    double x;
    double y;
    double heading; // in degrees

    // properties for INTAKE and LIFT
    int intakeLevel;
    int liftLevel = 600;

    double waitTime;

    boolean setIndividualAxis = false;

    // controllers for MOVE
    protected NavigationPID xPID;
    protected NavigationPID yPID;

    // default PID values
    double[] pidValues = {.152, .00165765, .0016622};

    protected Robot robot;

    // Basic constructor for this class
    public BaseAction(int id, Robot robot) {
        this.identity = id;
        this.robot = robot;
        timerReset = false;
        timer = new ElapsedTime();
    }


    protected void moveTo(){
        resetTimer();
        // Allows the robot to move one axis at a time.
        if (x == 0 && !setIndividualAxis){
            xPID.setTarget(robot.drive.getX());
            setIndividualAxis = true;
        }else if (y == 0 && !setIndividualAxis) {
            yPID.setError(robot.drive.getY());
            setIndividualAxis = true;
        }

        // Has the robot run to position and once it is within the range of error then the action
        // may end.
        boolean there = robot.drive.runToPosition(xPID, yPID);
        boolean timeOut = timer.milliseconds() > 4500;
        // the timeout boolean is a safety in-case it starts trying to get infinitely closer to
        // the target position.
        endAction = there || timeOut;
    }

    /**
     * Grabs the corresponding X and Y coordinates from the AutoPositionPresets class's arrays
     * based on what the camera saw and which side we start on. Once the coordinate is set the
     * action changes itself to move.
     */
    protected void moveToSpike(){
        // initializes the PIDs so they aren't null.
        setNavPID();

        // grabs targets, the targets are stored in arrays because it makes it easy to grab w/
        // a spike zone number
        if (robot.RED){
            x = RED_SPIKE[SPIKE_ZONE][0];
            y = RED_SPIKE[SPIKE_ZONE][1];

        }else {
            x = BLUE_SPIKE[SPIKE_ZONE][0];
            y = BLUE_SPIKE[SPIKE_ZONE][1];
        }

        // checks if we need to make any adjustments
        checkXSign();
        checkFarSide();

        // sets targets
        xPID.setTarget(x);
        yPID.setTarget(y);

        // switches itself to move so it can move to target.
        identity = MOVE;
    }

    /**
     * Basically the same as moveToSpike but with a different set of arrays
     */
    protected void moveToBackdrop(){
        setNavPID();

        if (robot.RED){
            x = RED_BACKDROP_POS[SPIKE_ZONE][0];
            y = RED_BACKDROP_POS[SPIKE_ZONE][1];

        }else {
            x = BLUE_BACKDROP_POS[SPIKE_ZONE][0];
            y = BLUE_BACKDROP_POS[SPIKE_ZONE][1];
        }
        checkXSign();
        checkFarSide();

        xPID.setTarget(x);
        yPID.setTarget(y);

        identity = MOVE;
    }

    protected void dropPixels(){
        resetTimer();

        robot.delivery.autoDropPixels(0.4);
        if (timer.milliseconds() > 550)
            robot.delivery.autoDropPixels(0.15);

        endAction = timer.milliseconds() > 600;
    }

    protected void runIntake(){
        robot.intake.setIntakeHeight(intakeLevel);

        resetTimer();

        if (timer.milliseconds() < 2550)
            robot.intake.pixelIn(1);
        else
            robot.intake.pixelIn(0);
        endAction = timer.milliseconds() > 2700;
    }

    /**
     * Runs the lift - good to go
     */
    protected void runLift(){
        // same as intake
        resetTimer();
        boolean atPos = robot.delivery.driveLiftToPosition(liftLevel, (int) timer.milliseconds());
        endAction = timer.milliseconds() > 2550 || atPos;
        if (endAction)
            robot.delivery.autoDriveLift(0);
    }

    // good to go
    protected void retractDelivery(){
        resetTimer();

        robot.delivery.autoDropPixels(0.15);
        robot.delivery.autoExtend(0);
        boolean atPos = robot.delivery.driveLiftToPosition(12, (int) timer.milliseconds());
        endAction = timer.milliseconds() > 2200 || atPos;
    }

    /**
     * drops pixel by reversing Intake
     */
    protected void placePixel(){
        robot.intake.setIntakeHeight(3);
        robot.intake.autoPixelOut();

        // safety feature to ensure the robot isn't driving across the field into another bot
        robot.drive.calculateDrivePowers(0,0,0);

        resetTimer();
        if(timer.milliseconds() > 750) {
            endAction = true;
            // another safety feature, but mostly to ensure power isn't being wasted.
            robot.intake.pixelIn(0);
        }
    }

    protected void extendDropper(){
        resetTimer();
        if(timer.milliseconds() < 1400)
            robot.delivery.autoExtend(0.4);
        endAction = timer.milliseconds() > 2000;
    }

    /**
     * Runs the camera for half a second and sets the number for SPIKE_MARK.
     */
    protected void detectSpikeMark(){
        if (!timerReset)
            robot.cam.detectProp();
        resetTimer();

        // detection function
        robot.cam.getSpikeZone();
        endAction = timer.milliseconds() > 500;
    }

    /**
     * Was meant to be implemented if AprilTags were implemented.
     */
    protected void alignBotToTag(){
        // looks for the required tag
        // requires the use of moving to align itself
        // runs the delivery
    }

    /**
     * waits out timer until timer is greater than or equal to the parameter wait time. Pauses all
     * robot functions while stopped
     */
    protected void waiting() {
        resetTimer();
        robot.drive.calculateDrivePowers(0,0,0);
        robot.intake.pixelIn(0);
        robot.delivery.driveLift(0);

        endAction = timer.milliseconds() > (waitTime * 1000);
    }

    protected void shutOffBot(){
        robot.shutOff();
    }

    /**
     * @return whether or not this action has been completed. If completed the ActionQueue will
     * replace the 0 item (current action) with the next one
     */
    public boolean isFinished(){
        if (endAction)
            robot.drive.calculateDrivePowers(0,0,0);
        return endAction;
    }

    private void resetTimer(){
        if (!timerReset){
            timer.reset();
            timerReset = true;
        }
    }

    protected void checkXSign(){
        if (robot.RED) {
            this.x *= -1;
        }
    }

    protected void checkFarSide(){
        if (farStart){
            y += BACK_ADJUST_Y;
        }
    }

    /**
     * Initializes the PID controllers so they don't throw a NullPointerException when
     * interacted with
     */
    protected void setNavPID(){
        double outPutLimit = 2;
        double integralLimit = 3650;

        xPID = new NavigationPID(pidValues);
        yPID = new NavigationPID(pidValues);

        xPID.setOutputLimits(outPutLimit);
        yPID.setOutputLimits(outPutLimit);

        xPID.setMaxIOutput(integralLimit);
        yPID.setMaxIOutput(integralLimit);
    }

    /**
     * Was meant to be a safety feature but I forgot what for.
     * @return Whether or not to stop.
     */
    public boolean isEmergencyStop(){
        if (endAction && identity == MOVE && xPID.getError() + yPID.getError() > 100)
            return true;
        return false;
    }
}
