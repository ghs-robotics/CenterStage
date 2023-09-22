package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;

@Autonomous
public class TestAuto extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, telemetry);
    AutoActionHandler actionHandler = new AutoActionHandler(robot, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        int cycle = 1;
        robot.init();

        // create list of actions to run
        actionHandler.add(MOVE, new ParamHandler(50, 50, 0.0));
        waitForStart();
        telemetry.addLine("queuing actions");

        //actionHandler.findAndSetZone();
        actionHandler.init();

        while (opModeIsActive()){
            actionHandler.run();
            actionHandler.status();
            telemetry.update();
        }
    }
}
