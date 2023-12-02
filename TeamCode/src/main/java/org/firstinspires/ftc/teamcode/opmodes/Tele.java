package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;
    TeleOpProfile gp1;
    TeleOpProfile gp2;

    boolean driveMode;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        gp1 = new TeleOpProfile(gamepad1, true);
        gp2 = new TeleOpProfile(gamepad2, false);

        driveMode = false;

        robot.init();
        waitForStart();
        telemetry.addLine("Initializing");
        robot.update();

        while (opModeIsActive()){
            gp1.update();
            gp2.update();

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 1
            //-------------------------------------------------------------------------------------

            // toggle drive mode. True is metaDrive, False is regular drive - left bumper
            if(gp1.driveMode)
                driveMode = !driveMode;

            // driving
            robot.drive.calculateDrivePowers(gp1.drivingX, gp1.drivingY, gp1.drivingRot, driveMode);

            // runs hang servos and winds the string - dpad down and dpad up
            robot.hang.hang(gp1.loweringHanging, gp1.raisingHanging);

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------

            // available buttons dpad_left, dpad_right

            // changes intake height - left bumper and right bumper
            robot.intake.changeIntakeHeight(gp2.raisingIntake, gp2.loweringIntake);

            // runs intake - left and right trigger
            robot.intake.pixelIn(gp2.right_trigger - gp2.left_trigger);

            // extends outtake - right joystick y-axis
            robot.delivery.setExtensionPosition(gp2.extendOuttake);

            // changes drop servo position - b
            robot.delivery.setDropPosition(gp2.dropPixel);

            // drives lift - left joystick, y-axis
            robot.delivery.driveLift(gp2.driveLift);

            // Launches drone - a button
            robot.drone.launchDrone(gp2.launchDrone);

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
