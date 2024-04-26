package org.firstinspires.ftc.teamcode.opmodes_teleop.input;

public class Button {
    private boolean lastState;
    private boolean currentState;

    public void update(boolean state){
        lastState = currentState;
        currentState = state;
    }

    public boolean pressing(){
        return currentState;
    }

    // checks if there's a difference between the current and last state
    // best way to check if this is working is to write a counter in teleOp where everytime a button
    // is pressed the counter increases by 1.
    public boolean pressed(){
        return lastState != currentState && !lastState;
    }

    // pls use this one day ;-;
    // it is the equal and opposite of pressed()
    public boolean released(){
        return lastState != currentState && lastState;
    }
}
