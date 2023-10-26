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

        // create list of actions to run ------------------------------------------
        int cycle = 1;

        actionHandler.add(MOVE, new ParamHandler());
        for (int i = 0; i < cycle; i++) {
            actionHandler.add(DELIVER);
            actionHandler.add(MOVE, new ParamHandler());
            actionHandler.add(INTAKE);
            actionHandler.add(MOVE, new ParamHandler());
        }
        actionHandler.add(MOVE, new ParamHandler());
        actionHandler.add(PLACE);
        actionHandler.add(MOVE);

        // list stops here --------------------------------------------------------

        telemetry.addLine("queuing actions");
        telemetry.addLine(actionHandler.getTotalActions() + " total actions");
        telemetry.update();

        waitForStart();


        actionHandler.findAndSetZone();
        actionHandler.init();

        while (opModeIsActive()){
            actionHandler.run();
            actionHandler.status();
            telemetry.update();
        }
    }
}