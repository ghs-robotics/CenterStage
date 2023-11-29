package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

public class TuneLift extends LinearOpMode {
    Robot robot;
    Controller gp;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        gp = new Controller(gamepad2);

        robot.init();
        waitForStart();
        telemetry.addLine("Initializing");
        robot.update();

        while (opModeIsActive()) {
            gp.update();

            robot.delivery.driveLiftMotor1(gp.left_stick_y);
            robot.delivery.driveLiftMotor2(gp.right_stick_y);

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
