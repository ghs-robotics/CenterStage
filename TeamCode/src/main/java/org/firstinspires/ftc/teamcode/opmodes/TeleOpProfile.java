package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

import java.util.ArrayList;

public class TeleOpProfile extends Controller {
    private int name;

    private final String[] driverName;
    private final String[] opName;

    public boolean driverOp;

    public ArrayList<Boolean> teleBooleans = new ArrayList<Boolean>();
    public ArrayList<Double> teleDoubles = new ArrayList<Double>();

    public boolean driveMode;
    public double drivingX;
    public double drivingY;
    public double drivingRot;

    public boolean loweringHanging;
    public boolean raisingHanging;

    public boolean raisingIntake;
    public boolean loweringIntake;

    public double pixelIn;
    public boolean dropPixel;

    public double extendOuttake;

    public double driveLift;
    public boolean liftSetHeight;
    public boolean liftToPosition;

    public boolean launchDrone;

    public TeleOpProfile(Gamepad gamepad, boolean driverOp) {
        super(gamepad);
        this.driverOp = driverOp;

        name = 0;
        driverName = new String[] {"Lillian"};
        opName = new String[] {"Ivan"};

        if (driverOp) {
            setLillianGamepad1();
        } else {
            setIvanGamepad2();
        }
    }

    public void setLillianGamepad1() {
        driveMode = left_bumper.pressed();
        drivingX = left_stick_x;
        drivingY = left_stick_y;
        drivingRot = right_stick_x;
        loweringHanging = dpad_down.pressing();
        raisingHanging = dpad_up.pressing();
    }

    public void setIvanGamepad2() {
        loweringIntake = left_bumper.pressed();
        raisingIntake = right_bumper.pressed();
        pixelIn = right_trigger - left_trigger;
        dropPixel = b.pressed();
        driveLift = left_stick_y;
        extendOuttake = right_stick_y;
        liftSetHeight = dpad_down.pressed();
        liftToPosition = y.pressed();
        launchDrone = a.pressed();
    }

    public void addToList () {
        // gamepad 1
        teleBooleans.add(driveMode); // 0
        teleBooleans.add(loweringHanging); // 1
        teleBooleans.add(raisingHanging); // 2

        teleDoubles.add(drivingX); // 0
        teleDoubles.add(drivingY); // 1
        teleDoubles.add(drivingRot); // 2

        // gamepad 2
        teleBooleans.add(loweringIntake); // 3
        teleBooleans.add(raisingIntake); // 4
        teleBooleans.add(dropPixel); // 5
        teleBooleans.add(liftSetHeight); // 6
        teleBooleans.add(liftToPosition); // 7
        teleBooleans.add(launchDrone); // 8

        teleDoubles.add(pixelIn); // 3
        teleDoubles.add(driveLift); // 4
        teleDoubles.add(extendOuttake); // 5
    }

    public void update() {
        super.update();
        if (driverOp) {
            switch (name) {
                case 0:
                    setLillianGamepad1();
                    break;
            }
        } else {
            switch (name) {
                case 0:
                    setIvanGamepad2();
                    break;
            }
        }
    }
}
