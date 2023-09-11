package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.INTAKE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.PLACE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoRunner;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;

@Autonomous
public class Auto extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, telemetry);
    AutoRunner runner = new AutoRunner(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        int cycle = 1;
        robot.init();

        // create list of actions to run
        runner.add(MOVE, new ParamHandler(MOVE));
        for (int i = 0; i < cycle; i++) {
            runner.add(DELIVER);
            runner.add(MOVE, new ParamHandler(MOVE));
            runner.add(INTAKE);
            runner.add(MOVE, new ParamHandler(MOVE));
        }
        runner.add(MOVE, new ParamHandler(MOVE));
        runner.add(PLACE);
        runner.add(MOVE);

        waitForStart();

        runner.init();

        while (opModeIsActive()){
            runner.run();
            telemetry.update();
        }
    }
}
