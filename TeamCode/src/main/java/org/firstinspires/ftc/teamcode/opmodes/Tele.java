package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;

@TeleOp
public class Tele extends LinearOpMode {
    Robot robot;
    TeleOpProfile gp1;
    TeleOpProfile gp2;

    boolean driveMode;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        gp1 = new TeleOpProfile(gamepad1, true);
        gp2 = new TeleOpProfile(gamepad2, false);

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

            if(gp1.teleBooleans.get(0)) {
                driveMode = !driveMode;
            }

            robot.drive.calculateDrivePowers(gp1.teleDoubles.get(0),
                    gp1.teleDoubles.indexOf(1), gp1.teleDoubles.indexOf(2), driveMode);

            robot.hang.hang(gp1.teleBooleans.get(1), gp1.teleBooleans.get(2));

            //-------------------------------------------------------------------------------------
            //                                  GAMEPAD 2
            //-------------------------------------------------------------------------------------

            robot.intake.changeIntakeHeight(gp2.teleBooleans.get(3), gp2.teleBooleans.get(4));

            robot.intake.pixelIn(gp2.teleDoubles.get(3));

            robot.delivery.changeDropPosition(gp2.teleBooleans.get(5));

            robot.delivery.driveLift(gp2.teleDoubles.get(4));

            robot.delivery.setExtensionPower(gp2.teleDoubles.get(5));

            robot.delivery.changeLiftHeight(gp2.teleBooleans.get(6), gp2.teleBooleans.get(7));

            robot.delivery.setRunLiftToPosition(gp2.teleBooleans.get(7));

            robot.drone.launchDrone(gp2.teleBooleans.get(8));

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------

            robot.update();
            robot.getTeleOpTelemetry();
        }
    }
}
