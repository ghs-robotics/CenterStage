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
import org.firstinspires.ftc.teamcode.cv.TeamPropPipeline;

@Autonomous
public class Auto extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, telemetry);
    AutoRunner runner = new AutoRunner(robot);

    private int spikeMarkPos = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        int cycle = 1;
        robot.init();

        //camera
        robot.cam.init();

        /*// create list of actions to run
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

        runner.init();*/

        while (opModeIsActive()){
            //runner.run();
            TeamPropPipeline tpp = robot.cam.TPPipeline;
            if (tpp.detectionFinished)
                if (tpp.finalSpikeMarkPos != -1) spikeMarkPos = tpp.finalSpikeMarkPos;

            telemetry.addData("spikeMarkPos: ", spikeMarkPos);
            telemetry.addLine();

            telemetry.update();
        }
    }
}
