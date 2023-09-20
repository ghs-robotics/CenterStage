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
    Robot robot = new Robot(hardwareMap, telemetry);
    AutoActionHandler actionHandler = new AutoActionHandler(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        int cycle = 1;
        robot.init();

        // create list of actions to run
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

        waitForStart();

        actionHandler.init();

        while (opModeIsActive()){
            actionHandler.run();
            telemetry.update();
        }
    }
}
