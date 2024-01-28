package org.firstinspires.ftc.teamcode.opmodes_teleop;

import static org.firstinspires.ftc.teamcode.control.cv.Camera.SPIKE_ZONE;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;
import org.firstinspires.ftc.teamcode.opmodes_teleop.input.Controller;

@TeleOp
public class TestTele extends LinearOpMode {
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
            robot.drive.calculateDrivePowers(-gp1.left_stick_x, -gp1.left_stick_y, gp1.right_stick_x);
//            robot.drive.calculateDrivePowers(new PoseVelocity2d
//                    (new Vector2d(-gp1.left_stick_x, -gp1.left_stick_y), gp1.right_stick_x));

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
