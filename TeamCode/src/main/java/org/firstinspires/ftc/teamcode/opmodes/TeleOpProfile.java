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

        public void addToList1 () {
            // gamepad 1
            teleBooleans.add(driveMode); // 0
            teleBooleans.add(loweringHanging); // 1
            teleBooleans.add(raisingHanging); // 2

            teleDoubles.add(drivingX); // 0
            teleDoubles.add(drivingY); // 1
            teleDoubles.add(drivingRot); // 2
        }

        public void addToList2 () {
        // gamepad 2
        teleBooleans.add(loweringIntake); // 0
        teleBooleans.add(raisingIntake); // 1
        teleBooleans.add(dropPixel); // 2
        teleBooleans.add(liftSetHeight); // 3
        teleBooleans.add(liftToPosition); // 4
        teleBooleans.add(launchDrone); // 5

        teleDoubles.add(pixelIn); // 0
        teleDoubles.add(driveLift); // 1
        teleDoubles.add(extendOuttake); // 2
    }

    public void updateList1 () {
        teleBooleans.set(0, driveMode);
        teleBooleans.set(1, loweringIntake);
        teleBooleans.set(2, raisingIntake);

        teleDoubles.set(0, drivingX);
        teleDoubles.set(1, drivingY);
        teleDoubles.set(2, drivingRot);
    }

    public void updateList2 () {
        teleBooleans.set(0, loweringIntake);
        teleBooleans.set(1, raisingIntake);
        teleBooleans.set(2, dropPixel);
        teleBooleans.set(3, liftSetHeight);
        teleBooleans.set(4, liftToPosition);
        teleBooleans.set(5, launchDrone);

        teleDoubles.set(0, pixelIn);
        teleDoubles.set(1, driveLift);
        teleDoubles.set(2, extendOuttake);
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
