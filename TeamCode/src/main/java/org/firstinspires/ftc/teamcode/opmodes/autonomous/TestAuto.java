package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.WAIT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;

@Autonomous
public class TestAuto extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        actionHandler = new AutoActionHandler(robot, telemetry);
        robot.init();


        // create list of actions to run
//                0.0, true));
//        actionHandler.add(AutoPresets.getRouteA(robot, telemetry));
//        actionHandler.add(MOVE, 50, 50, 0.0);
//        actionHandler.add(MOVE, 0, 50, 0.0);
//        actionHandler.add(MOVE, 0, 0, 0.0);
        actionHandler.add(LIFT, 250);
//        actionHandler.add(WAIT, 20);


//        telemetry.addLine("queuing actions");
//        telemetry.addLine(actionHandler.getTotalActions() + " total actions");
        robot.getAutoTelemetry();

        actionHandler.init();
        waitForStart();
        //actionHandler.findAndSetZone();

        while (opModeIsActive()){
            actionHandler.run();
            robot.update();
            robot.getAutoTelemetry();
            telemetry.addLine();
            actionHandler.status();
            //robot.getAutoTelemetry();
        }

    }
}
