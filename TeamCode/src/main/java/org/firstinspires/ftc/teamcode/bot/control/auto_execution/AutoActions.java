package org.firstinspires.ftc.teamcode.bot.control.auto_execution;


public class AutoActions {
    // identities
    public static final int MOVE = 0;
    public static final int INTAKE = 1;
    public static final int DELIVER = 2;
    public static final int PLACE = 4;
    public static final int ALIGN = 5;

    private int identity;
    private boolean endAction;

    private int zone;
    ParamHandler params;

    public AutoActions(int id){
        this.identity = id;
    }

    public AutoActions(int id, ParamHandler params){
        this.identity = id;
        this.params = params;
    }

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

    private void moveTo(){
        // get current pos
        // calculate diff
        // set power
        // repeat until diff is 0 or in an acceptable range
        // set endAction to true
    }

    private void runIntake(){
        // start runIntake
        // use timer to make sure the bot has enough time to intake
    }

    private void runDelivery(){
        // same as intake
    }

    private void detectElement(){
        // run cv to look for the element's placement and then move accordingly
        // if completed this function should return something
    }

    private void placePixel(){
        // drops the pixel
    }

    private void alignBotToTag(){
        // looks for the required tag
        // requires the use of moving to align itself
        // runs the delivery

    }

    public boolean isFinished(){
        return endAction;
    }

    // checks which action this object will execute
    public int getIdentity() {
        return identity;
    }

    public int getZone(){
        return zone;
    }
}
