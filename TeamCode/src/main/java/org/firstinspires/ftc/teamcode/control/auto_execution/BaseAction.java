package org.firstinspires.ftc.teamcode.control.auto_execution;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.cv.Camera.SPIKE_ZONE;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.centerBackDropPos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.centerSpikePos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.leftBackDropPos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.leftSpikePos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.rightBackDropPos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.rightSpikePos;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.originalPID.NavigationPID;

public class BaseAction {

    protected int identity;
    protected boolean endAction;

    protected ElapsedTime timer;
    protected boolean timerReset;

    protected String description;

    double x;
    double y;
    double heading; // in degrees

    int intakeLevel;
    int liftLevel = 650;

    double waitTime;

    protected NavigationPID xPID;
    protected NavigationPID yPID;

    double[] pidValues = {.152, .00165765, .0016622};


    protected Robot robot;


    public BaseAction(int id, Robot robot) {

        this.identity = id;
        this.robot = robot;
        timerReset = false;
        timer = new ElapsedTime();
    }


    protected void moveTo(){
        resetTimer();

//        boolean there = robot.drive.runToPosition(xPID, yPID);
//        boolean timeOut = timer.milliseconds() > 4250;
//        endAction = there || timeOut;
    }

    protected void moveToSpike(){
        setNavPID();
        if (SPIKE_ZONE == 1){
            x = leftSpikePos[0];
            if (robot.RED)
                yPID.setTarget(leftSpikePos[1]);
            else
                yPID.setTarget(rightSpikePos[1]);
        } else if (SPIKE_ZONE == 2) {
            x = (centerSpikePos[0]);
            yPID.setTarget(centerSpikePos[1]);
        } else {
            x = (rightSpikePos[0]);
            if (robot.RED)
                yPID.setTarget(rightSpikePos[1]);
            else
                yPID.setTarget(leftSpikePos[1]);
        }

        checkXSign();

        xPID.setTarget(x);

        identity = MOVE;
    }

    protected void moveToBackdrop(){
        setNavPID();
        if (SPIKE_ZONE == 1){
            x = (leftBackDropPos[0]);
            if (robot.RED)
                yPID.setTarget(leftBackDropPos[1]);
            else
                yPID.setTarget(rightBackDropPos[1]);
        } else if (SPIKE_ZONE == 2 && robot.RED) {
            x = (centerBackDropPos[0]);
            yPID.setTarget(centerBackDropPos[1]);
        } else {
            x = (rightBackDropPos[0]);
            if (robot.RED)
                yPID.setTarget(rightBackDropPos[1]);
            else
                yPID.setTarget(leftBackDropPos[1]);
        }

        checkXSign();

        xPID.setTarget(x);

        identity = MOVE;
    }

    protected void dropPixels(){
        resetTimer();

        robot.delivery.autoDropPixels(0.4);
        if (timer.milliseconds() > 300)
            robot.delivery.autoDropPixels(0.15);

        endAction = timer.milliseconds() > 400;
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
        endAction = timer.milliseconds() > 3750 || atPos;
        if (atPos)
            robot.delivery.autoDriveLift(0);
    }

    // good to go

    protected void retractDelivery(){
        resetTimer();

        robot.delivery.autoDropPixels(0.15);
        robot.delivery.autoExtend(0);
        boolean atPos = robot.delivery.driveLiftToPosition(10, (int) timer.milliseconds());
        endAction = timer.milliseconds() > 2200 || atPos;
    }

    /**
     * drops pixel by reversing Intake
     */
    protected void placePixel(){
        robot.intake.setIntakeHeight(3);
        robot.intake.autoPixelOut();

        resetTimer();
        if(timer.milliseconds() > 2500) {
            endAction = true;
            robot.intake.pixelIn(0);
        }
    }

    protected void extendDropper(){
        resetTimer();
        if(timer.milliseconds() < 1400)
            robot.delivery.autoExtend(0.4);
        endAction = timer.milliseconds() > 2000;
    }

    protected void detectSpikeMark(){
        if (!timerReset)
            robot.cam.detectProp();
        resetTimer();
        robot.cam.getSpikeZone();
        endAction = timer.milliseconds() > 2100;
    }

    protected void alignBotToTag(){
        // looks for the required tag
        // requires the use of moving to align itself
        // runs the delivery
    }

    /**
     * waits out timer until timer is greater than or equal to the parameter wait time
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
     * @return whether or not this action has been completed
     */
    public boolean isFinished(){
        return endAction;
    }

    private void resetTimer(){
        if (!timerReset){
            timer.reset();
            timerReset = true;
        }
    }

    protected void checkXSign(){
        if (!robot.RED)
            this.x *= -1;

    }

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
}
