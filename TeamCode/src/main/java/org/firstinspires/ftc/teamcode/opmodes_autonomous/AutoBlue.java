package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DETECT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE_TO_SPIKE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.PLACE_PIXEL;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.WAIT;
import static org.firstinspires.ftc.teamcode.control.presets.AutoPresets.goToSpikeMark;

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

        // create list of actions to run  ------------------------------------------------------------
        actionHandler.add(goToSpikeMark(robot, telemetry));
        actionHandler.add(WAIT, 0.1);
        actionHandler.add(MOVE, 600, 700, 0);
        actionHandler.add(DELIVER);
        actionHandler.add(WAIT, 0.1);
        actionHandler.add(MOVE, 200, 775, 0);

        // don't queue past this line. ---------------------------------------------------------------
        actionHandler.init();
        waitForStart();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("queuing actions");
            telemetry.addLine(actionHandler.getTotalActions() + " total actions");
            telemetry.update();
        }

        while (opModeIsActive()){
            actionHandler.run();
            actionHandler.status();
        }
    }
}