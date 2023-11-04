package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.INTAKE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.PLACE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;

@Autonomous
public class  Auto extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        actionHandler = new AutoActionHandler(robot, telemetry);
        robot.init();

        // create list of actions to run
        //actionHandler.add(WAIT, new ParamHandler(20));
        actionHandler.add(MOVE, new ParamHandler(0, -1500, 0.0));




        telemetry.addLine("queuing actions");
        telemetry.addLine(actionHandler.getTotalActions() + " total actions");

        waitForStart();
        //actionHandler.findAndSetZone();
        actionHandler.init();

        while (opModeIsActive()){
            actionHandler.run();
            actionHandler.status();
            robot.update();
            robot.getTelemetry();
        }
    }
}