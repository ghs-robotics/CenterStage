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

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);

        waitForStart();
        telemetry.addLine("Initializing");
        telemetry.update();

        while (opModeIsActive()){
            gp1.update();
            gp2.update();

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 1
            //-------------------------------------------------------------------------------------
            //robot.drive.calculateDrivePowers(gp1.left_stick_x, gp1.left_stick_y, gp1.right_stick_x);

            robot.blinkin.runLeds();
            robot.blinkin.numPixels1(gp1.a.pressed());
            robot.blinkin.numPixels2(gp1.b.pressed());
            robot.blinkin.numPixelsReset(gp1.x.pressed());



            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------


            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //------------------------------------------------------------------------------------
            telemetry.update();
        }

    }
}
