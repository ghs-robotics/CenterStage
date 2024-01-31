package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.WAIT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActionHandler;

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
//                0.0, true));
        //left
//        actionHandler.add(DETECT);
        actionHandler.add(MOVE, -600, 725, 0);
        actionHandler.add(WAIT, 2);
        actionHandler.add(LIFT);
        actionHandler.add(EXTEND);
        actionHandler.add(DROP);
        actionHandler.add(RETRACT);
        actionHandler.add(MOVE, -200, 730, 0);

//        actionHandler.add(LIFT);
//        actionHandler.add(DELIVER);
//        actionHandler.add(RETRACT);
//        actionHandler.add(MOVE, -100, -600, 0);


//        telemetry.addLine("queuing actions");
//        telemetry.addLine(actionHandler.getTotalActions() + " total actions");


        robot.getAutoTelemetry();

        actionHandler.init();
        waitForStart();
        //actionHandler.findAndSetZone();

        while (opModeIsActive()){
            actionHandler.run();
            telemetry.addLine();
            actionHandler.status();
            //robot.getAutoTelemetry();
        }
    }
}