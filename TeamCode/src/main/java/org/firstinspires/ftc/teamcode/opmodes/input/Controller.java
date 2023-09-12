package org.firstinspires.ftc.teamcode.opmodes.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class Controller {
    private Gamepad gamepad;

    public Button a = new Button();
    public Button b = new Button();
    public Button x = new Button();
    public Button y = new Button();
    public Button left_bumper = new Button();
    public Button right_bumper = new Button();
    public Button dpad_left = new Button();
    public Button dpad_right = new Button();
    public Button dpad_up = new Button();
    public Button dpad_down = new Button();

    public double left_stick_x;
    public double left_stick_y;
    public double right_stick_x;
    public double right_stick_y;
    public double left_trigger;
    public double right_trigger;

    ArrayList<Button> buttons;
    ArrayList<Boolean> gamepadButtons;


    public Controller(Gamepad gamepad){
        this.gamepad = gamepad;

        buttons = new ArrayList<Button>();
        gamepadButtons = new ArrayList<Boolean>();

        buttons.add(a);
        buttons.add(b);
        buttons.add(x);
        buttons.add(y);
        buttons.add(left_bumper);
        buttons.add(right_bumper);
        buttons.add(dpad_left);
        buttons.add(dpad_right);
        buttons.add(dpad_up);
        buttons.add(dpad_down);

        update();
    }

    public void update(){
        updateButtons();

        for (int i = 0; i < buttons.size(); i++)
            buttons.get(i).update(gamepadButtons.get(i));

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;

    }

    private void updateButtons(){
        gamepadButtons.clear();

        gamepadButtons.add(gamepad.a);
        gamepadButtons.add(gamepad.b);
        gamepadButtons.add(gamepad.x);
        gamepadButtons.add(gamepad.y);
        gamepadButtons.add(gamepad.left_bumper);
        gamepadButtons.add(gamepad.right_bumper);
        gamepadButtons.add(gamepad.dpad_left);
        gamepadButtons.add(gamepad.dpad_right);
        gamepadButtons.add(gamepad.dpad_up);
        gamepadButtons.add(gamepad.dpad_down);
    }


}
