package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.bot.control.Navigation.TICKS_PER_TILE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.WAIT;

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
//        actionHandler.add(MOVE, new ParamHandler((TICKS_PER_TILE), (int) -(TICKS_PER_TILE * 1.3), 0.0));
        actionHandler.add(LIFT);
        actionHandler.add(EXTEND);
        actionHandler.add(DROP);
        actionHandler.add(RETRACT);
//        actionHandler.add(MOVE, new ParamHandler(100, (int) -(TICKS_PER_TILE * 1.3), 0.0));
//        actionHandler.add(MOVE, new ParamHandler(100, (int) -(TICKS_PER_TILE * 1.7), 0.0));




        telemetry.addLine("queuing actions");
        telemetry.addLine(actionHandler.getTotalActions() + " total actions");

        waitForStart();
        //actionHandler.findAndSetZone();
        actionHandler.init();

        while (opModeIsActive()){
            actionHandler.run();
            actionHandler.status();
            robot.update();
        }

    }
}
