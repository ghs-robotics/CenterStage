package org.firstinspires.ftc.teamcode.control.auto_execution;

import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.centerBackDropPos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.centerSpikePos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.leftBackDropPos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.leftSpikePos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.rightBackDropPos;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.rightSpikePos;
import static org.firstinspires.ftc.teamcode.control.cv.Camera.SPIKE_ZONE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.originalPID.NavigationPID;

public class AutoActions extends BaseAction {
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

    public AutoActions(int id, Robot robot){
        super(id, robot);
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
        else if (id == WAIT){
            waitTime = value;
        }
    }
    public AutoActions (int id, Robot robot, double[] pos){
        this(id, robot, (int) pos[0], (int) pos[1], (int) pos[2]);
    }
    /**
     * Driving the rob
     */
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
                retractDelivery();
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

    /**
     * @return the identity of this action
     */
    public int getIdentity(){
        return identity;
    }


    public int getTimer() {
        return (int) (timer.milliseconds() / 1000);
    }
}
