package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.opmodes_teleop.input.Controller;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;

    Intake intake;
    Controller gp1;
    Controller gp2;
    Delivery deliver;

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
            if(gp1.left_bumper.pressed()) {
                driveMode = !driveMode;
            }


            // driving
            robot.drive.calculateDrivePowers(-gp1.left_stick_x, -gp1.left_stick_y,
                    gp1.right_stick_x, driveMode);


            // launches drone - a button
            robot.drone.launchDrone(gp1.x.pressing());

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------

            robot.intake.changeIntakeHeight(gp2.left_bumper.pressed(),gp2.right_bumper.pressed());


            robot.intake.pixelIn(gp2.right_trigger - gp2.left_trigger);


            robot.delivery.changeExtensionLength(gp2.dpad_left.pressed(), gp2.dpad_right.pressed());


            robot.delivery.driveLift(gp2.left_stick_y);


            robot.delivery.hangLift(gp2.left_stick_y, gp2.right_stick_y);


            robot.delivery.changeDropPosition(gp2.x.pressing());


            robot.delivery.setHangMode(gp2.a.pressed());


            robot.led.switchLedOnState(gp2.y.pressed());

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
