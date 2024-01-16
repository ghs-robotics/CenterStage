package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DETECT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE_TO_SPIKE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.PLACE_PIXEL;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.WAIT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActionHandler;

@Autonomous
public class AutoBlue extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        actionHandler = new AutoActionHandler(robot, telemetry);
        robot.init();

        // create list of actions to run
        actionHandler.add(DETECT);
        actionHandler.add(MOVE_TO_SPIKE);
        actionHandler.add(PLACE_PIXEL);
        actionHandler.add(WAIT, 1.5);
        actionHandler.add(MOVE, 600, 700, 0);
        actionHandler.add(LIFT);
        actionHandler.add(DROP);
        actionHandler.add(RETRACT);
        actionHandler.add(MOVE, 200, 705, 0);

//        actionHandler.add(AutoPresets.getBeginningNearBackDrop(robot, telemetry));
//        actionHandler.add(DETECT);
//        actionHandler.add(MOVE_TO_SPIKE);
//        actionHandler.add(PLACE_PIXEL);
//        actionHandler.add(MOVE_TO_BACKDROP);
//        actionHandler.add(LIFT);
//        actionHandler.add(DELIVER);
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