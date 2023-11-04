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

    boolean driveMode;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);

        driveMode = false;


        robot.init();
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
            if(gp1.left_bumper.pressed())
                driveMode = !driveMode;

            // driving
            robot.drive.calculateDrivePowers(gp1.left_stick_x, gp1.left_stick_y, gp1.right_stick_x, driveMode);

            // runs hang servos and winds the string - dpad down and dpad up
            robot.hang.hang(gp1.dpad_down.pressing(), gp1.dpad_up.pressing());

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------

            // available buttons y, a, dpad_left, dpad_right

            // changes intake height - left bumper and right bumper
            robot.intake.changeIntakeHeight(gp2.left_bumper.pressed(), gp2.right_bumper.pressed());

            // runs intake and conveyor belt - x
            robot.intake.pixelIn(gp2.x.pressing());

            // runs intake analogly - left and right trigger
            robot.intake.pixelIn(gp2.right_trigger - gp2.left_trigger);

            // changes drop servo position - b
            robot.delivery.changeDropPosition(gp2.b.pressed());

            // drives lift - left joystick, y-axis
            robot.delivery.driveLift(gp2.left_stick_y);

            // extends outtake - right joystick y-axis
            robot.delivery.extendOuttake(gp2.right_stick_y);

            // runs lift to set height - dpad up
            robot.delivery.changeLiftHeight(gp2.dpad_down.pressed(), gp2.dpad_up.pressed());

            // changes mode from driving lift to setting lift position or vice versa - dpad down
            robot.delivery.setRunLiftToPosition(gp2.y.pressed());

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------
            robot.update();
            robot.getTelemetry();
        }

    }
}
