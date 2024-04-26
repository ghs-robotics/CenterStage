package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.components.BallDrive;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes_teleop.input.Controller;

@TeleOp
public class MecanumDriveTele extends LinearOpMode{
    MecanumDrive robot;
    Gyro gyro;

    Controller gp1;
    Controller gp2;

    boolean driveMode;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = new Gyro(hardwareMap);
        robot = new MecanumDrive(hardwareMap);

        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);

        driveMode = false;

        waitForStart();
        telemetry.addLine("Initializing");

        while (opModeIsActive()){
            gp1.update();
            gp2.update();

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 1
            //-------------------------------------------------------------------------------------

            robot.calculateDrivePower(gp1.left_stick_x, gp1.left_stick_y, -gp1.right_stick_x);


        }
    }
}

