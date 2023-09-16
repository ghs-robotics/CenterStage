package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;
    Controller con1;
    Controller con2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        con1 = new Controller(gamepad1);
        con2 = new Controller(gamepad2);

        robot.init();

        telemetry.addLine("Initializing");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            con1.update();
            con2.update();
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

//            if (gamepad2.dpad_right) {
//                outtake.pixelOut();
//            }

//            if (gamepad2.a) {
//                lift.setLow();
//            }
//
//            if (gamepad2.x) {
//                lift.setMid();
//            }
//
//            if (gamepad2.y) {
//                lift.setHigh();
//            }
            telemetry.update();
        }
    }
}
