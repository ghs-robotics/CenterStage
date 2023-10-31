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
            robot.drive.calculateDrivePowers(gp1.left_stick_x, gp1.left_stick_y, gp1.right_stick_x);

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------
            robot.intake.changeIntakeHeight(gp2.left_bumper.pressed(), gp2.right_bumper.pressed());
            robot.intake.pixelIn(gp2.dpad_left.pressing());

            if (!gp2.dpad_up.pressed()) robot.deliver.driveLift(gp2.left_stick_y);
            robot.deliver.changeLiftHeight(gp2.dpad_up.pressed());
            robot.deliver.changeDropHeight(gp2.dpad_right.pressed());
            robot.deliver.extendOuttake(gp2.right_stick_y);

            robot.hang.hang(gp2.dpad_down.pressing(), gp2.dpad_up.pressing());

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------
            robot.update();
            robot.getTelemetry();
        }

    }
}
