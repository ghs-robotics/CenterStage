package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;

import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Lift;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Outtake;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;
    Intake intake;
    Lift lift;
    Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);


        waitForStart();
        telemetry.addLine("Initializing");
        telemetry.update();

        while (opModeIsActive()){

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 1
            //-------------------------------------------------------------------------------------
            robot.drive.calculateDrivePowers(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);





            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------
            if (gamepad2.dpad_left) {
                intake.pixelIn();
            }

            if (gamepad2.dpad_up) {
                lift.raiseLift();
            }

            if (gamepad2.dpad_right) {
                outtake.pixelOut();
            }

            if (gamepad2.a) {
                lift.setLow();
            }

            if (gamepad2.x) {
                lift.setMid();
            }

            if (gamepad2.y) {
                lift.setHigh();
            }
        }
    }
}
