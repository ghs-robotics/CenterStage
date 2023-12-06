package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;
     Controller gp1;
    Controller gp2;

    boolean driveMode;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);

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
            if(gp1.left_bumper.pressed())
                driveMode = !driveMode;


            // driving
            robot.drive.calculateDrivePowers(-gp1.left_stick_x, -gp1.left_stick_y,
                    gp1.right_stick_x, driveMode);


            robot.delivery.changeLiftMode(gp1.b.pressed());


            // changes drone mode - right bumper
            robot.drone.changeDroneMode(gp1.right_bumper.pressed());

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------

            // changes intake height - left bumper and right bumper
            robot.intake.changeIntakeHeight(gp2.left_bumper.pressed(),gp2.right_bumper.pressed());


            // runs intake - left and right trigger
            robot.intake.pixelIn(gp2.right_trigger - gp2.left_trigger);


            // extends outtake - left and right dpad
            robot.delivery.changeExtensionLength(gp2.dpad_left.pressed(), gp2.dpad_right.pressed());


//            robot.delivery.setExtendPower(gp2.right_stick_y);


            // drives lift - left joystick, y-axis
            robot.delivery.driveLift(gp2.left_stick_y);


            robot.delivery.tuneLiftDuringTele(gp2.left_stick_y, gp2.right_stick_y);


            // changes drop servo position - b
            robot.delivery.changeDropPosition(gp2.b.pressing());


            // launches drone - a button
            robot.drone.launchDrone(gp2.a.pressing());

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
