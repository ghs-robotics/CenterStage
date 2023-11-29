package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

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

        gp1.addToList1();
        gp2.addToList2();

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

            robot.intake.changeIntakeHeight(gp2.teleBooleans.get(0), gp2.teleBooleans.get(1));

            robot.intake.pixelIn(gp2.teleDoubles.get(0));

            robot.delivery.changeDropPosition(gp2.teleBooleans.get(2));

            robot.delivery.driveLift(gp2.teleDoubles.get(1));

            robot.delivery.setExtensionPower(gp2.teleDoubles.get(2));

            robot.delivery.changeLiftHeight(gp2.teleBooleans.get(3), gp2.teleBooleans.get(4));

            robot.delivery.setRunLiftToPosition(gp2.teleBooleans.get(4));

            robot.drone.launchDrone(gp2.teleBooleans.get(5));

            //-------------------------------------------------------------------------------------
            //                                  TELEMETRY
            //-------------------------------------------------------------------------------------

            robot.update();
            robot.getTeleOpTelemetry();
            gp1.updateList1();
            gp2.updateList2();
        }
    }
}
