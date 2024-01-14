package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.PLACE_PIXEL;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.WAIT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets.leftSpikePos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets;

@Autonomous
public class TestAuto extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        actionHandler = new AutoActionHandler(robot, telemetry);
        robot.init();


        // create list of actions to run
        actionHandler.add(PLACE_PIXEL);
        actionHandler.add(MOVE, leftSpikePos);


//        telemetry.addLine("queuing actions");
//        telemetry.addLine(actionHandler.getTotalActions() + " total actions");
        robot.getAutoTelemetry();

        actionHandler.init();
        waitForStart();
        //actionHandler.findAndSetZone();
        while (!opModeIsActive()){
            actionHandler.findAndSetZone();
            telemetry.addLine(String.valueOf(actionHandler.zone));
        }

        while (opModeIsActive()){
            actionHandler.run();
            robot.update();
            robot.getAutoTelemetry();
            telemetry.addLine();
            actionHandler.status();
            //robot.getAutoTelemetry();
        }

    }
}
