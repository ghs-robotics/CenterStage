package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

public class TeleOpProfile extends Controller {
    private int name;

    private final String[] driverName;
    private final String[] opName;

    public boolean driverOp;

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
    public boolean LiftToPosition;

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
        raisingIntake = left_bumper.pressed();
        loweringIntake = right_bumper.pressed();
        pixelIn = right_trigger - left_trigger;
        dropPixel = b.pressed();
        driveLift = left_stick_y;
        extendOuttake = right_stick_y;
        liftSetHeight = dpad_down.pressed();
        LiftToPosition = y.pressed();
        launchDrone = a.pressed();
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
