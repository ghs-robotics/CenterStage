package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.components.BallDrive;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Delivery;
import org.firstinspires.ftc.teamcode.bot.components.pixel_delivery.Intake;
import org.firstinspires.ftc.teamcode.opmodes_teleop.input.Controller;

@TeleOp
public class BallDriveTest extends LinearOpMode {
    BallDrive robot;
    Gyro gyro;

    Controller gp1;
    Controller gp2;

    public boolean onlyDrive;
    boolean driveMode;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = new Gyro(hardwareMap);
        robot = new BallDrive(hardwareMap, gyro);

        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);

        driveMode = false;

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

            if (gp1.a.pressed()){
                onlyDrive = !onlyDrive;
            }



            // driving

            robot.calculateDrivePowers(-gp1.left_stick_x, -gp1.left_stick_y,
                    gp1.right_stick_x, driveMode);

            if (onlyDrive) {
                robot.update();
                telemetry.addData("only driving on: ", onlyDrive);
                continue;
            }

            robot.update();

        }
    }
}
