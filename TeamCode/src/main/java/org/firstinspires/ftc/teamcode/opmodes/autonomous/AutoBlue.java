package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE_TO_SPIKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets;

@Autonomous
public class AutoBlue extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        actionHandler = new AutoActionHandler(robot, telemetry);
        robot.init();

        // create list of actions to run
//        actionHandler.add(AutoPresets.getBeginningNearBackDrop(robot, telemetry));
//        actionHandler.add(RETRACT);
        actionHandler.add(MOVE_TO_SPIKE);




        telemetry.addLine("queuing actions");
        telemetry.addLine(actionHandler.getTotalActions() + " total actions");

        waitForStart();
        //actionHandler.findAndSetZone();
        actionHandler.init();

        while (opModeIsActive()){
            actionHandler.run();
            actionHandler.status();
            robot.update();
//            robot.getTelemetry();
        }
    }
}