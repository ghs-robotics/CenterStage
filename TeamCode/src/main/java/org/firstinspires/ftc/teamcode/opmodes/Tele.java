package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;

import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Lift;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Outtake;
import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;
    Intake intake;
    Lift lift;
    Outtake outtake;
    Controller con1;
    Controller con2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
//        lift = new Lift(hardwareMap, telemetry);
//        outtake = new Outtake(hardwareMap, telemetry);
        con1 = new Controller(gamepad1);
        con2 = new Controller(gamepad2);

        waitForStart();
        telemetry.addLine("Initializing");
        telemetry.update();

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
            intake.pixelIn(con2.dpad_left.pressing());
            telemetry.addData("intake", con2.dpad_left.pressing());

            intake.changeIntakeHeight(con2.left_bumper.pressed(), con2.right_bumper.pressed());

            telemetry.addData("servo down", con2.left_bumper.pressed());
            telemetry.addData("servo up", con2.right_bumper.pressed());

            telemetry.addData("intakeServoPos", intake.getServoPos());

//            if (gamepad2.dpad_up) {
//                lift.raiseLift();
//            }

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
