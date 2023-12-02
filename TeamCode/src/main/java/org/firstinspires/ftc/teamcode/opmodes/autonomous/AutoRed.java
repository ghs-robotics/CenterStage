package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.bot.control.Navigation.TICKS_PER_TILE_X;
import static org.firstinspires.ftc.teamcode.bot.control.Navigation.TICKS_PER_TILE_Y;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets;

@Autonomous
public class AutoRed extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, true);
        actionHandler = new AutoActionHandler(robot, telemetry);
        robot.init();

        // create list of actions to run
//        actionHandler.add(AutoPresets.getBeginningNearBackDrop(robot, telemetry));
        actionHandler.add(AutoPresets.getRouteA(robot, telemetry));
//        actionHandler.add(MOVE, new ParamHandler(3 * TICKS_PER_TILE_X, -860, 0.0));
//        actionHandler.add(RETRACT);




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