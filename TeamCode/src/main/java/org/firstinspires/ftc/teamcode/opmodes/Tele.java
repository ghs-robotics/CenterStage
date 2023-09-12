package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        robot.init();

        telemetry.addLine("Initializing");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 1
            //-------------------------------------------------------------------------------------
            robot.drive.calculateDrivePowers(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);





            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------



            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------
            robot.update();
            robot.getTelemetry();
            telemetry.update();
        }

    }
}
