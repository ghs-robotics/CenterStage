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

            // changes drone mode - right bumper
            robot.drone.changeDroneMode(gp1.right_bumper.pressed());

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------

            // changes intake height - left bumper and right bumper
            robot.intake.changeIntakeHeight(gp2.raisingIntake, gp2.loweringIntake);


            // runs intake - left and right trigger
            robot.intake.pixelIn(gp2.right_trigger - gp2.left_trigger);


//            // extends outtake - left and right dpad
//            robot.delivery.changeExtensionLength(gp2.retractOuttake ,gp2.extendOuttake);

            robot.delivery.setExtendPower(gp2.powerExtension);

            // changes drop servo position - b
            robot.delivery.changeDropPosition(gp2.dropPixel);


            // drives lift - left joystick, y-axis
            robot.delivery.driveLift(gp2.driveLift);

            // launches drone - a button
            robot.drone.launchDrone(gp2.launchDrone);

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
