package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;

@Autonomous
public class TestAuto extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        actionHandler = new AutoActionHandler(robot, telemetry);
        robot.init();

        // create list of actions to run
        actionHandler.add(MOVE, new ParamHandler(100, 100, 0.0));
        actionHandler.add(MOVE, new ParamHandler(-100, 100, 0.0));
        actionHandler.add(MOVE, new ParamHandler(-100, -100, 0.0));
        actionHandler.add(MOVE, new ParamHandler(100, -100, 0.0));

        waitForStart();
        telemetry.addLine("queuing actions");
        telemetry.addLine(actionHandler.getTotalActions() + " total actions");

        //actionHandler.findAndSetZone();
        actionHandler.init();

        while (opModeIsActive()){
            if (!actionHandler.isFinished())
                actionHandler.run();
            actionHandler.status();
            telemetry.update();
        }
    }
}
