package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

@TeleOp
public class TuneLift extends LinearOpMode {
    Robot robot;
    Controller gp2;

@Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        gp2 = new Controller(gamepad2);

        robot.init();
        waitForStart();
        telemetry.addLine("Initializing");
        robot.update();

        while (opModeIsActive()) {
            gp2.update();

            robot.delivery.driveLiftMotor1(gp2.left_stick_y);
            robot.delivery.driveLiftMotor2(gp2.right_stick_y);

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
