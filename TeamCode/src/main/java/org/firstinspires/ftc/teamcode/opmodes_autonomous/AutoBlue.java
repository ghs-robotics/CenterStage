package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.WAIT;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.CENTER_PARKING;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.CORNER_PARKING;
import static org.firstinspires.ftc.teamcode.presets.AutoPresets.basicShort;
import static org.firstinspires.ftc.teamcode.presets.AutoPresets.goToSpikeMark;

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
        actionHandler.add(basicShort(robot, telemetry));

//        actionHandler.add(MOVE, CORNER_PARKING);
        actionHandler.add(MOVE, CENTER_PARKING);


        // don't queue past this line. ---------------------------------------------------------------
        actionHandler.init();
        waitForStart();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("queuing actions");
            telemetry.addLine(actionHandler.getTotalActions() + " total actions");
            telemetry.update();
        }

        while (opModeIsActive()){
            actionHandler.update();
        }
    }
}