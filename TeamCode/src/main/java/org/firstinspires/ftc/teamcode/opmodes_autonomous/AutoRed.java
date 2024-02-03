package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.WAIT;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.CENTER_PARKING;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.HALF_TO_MARK;
import static org.firstinspires.ftc.teamcode.presets.AutoPresets.basicShort;

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
        actionHandler.add(basicShort(robot, telemetry));

//        actionHandler.add(MOVE, CORNER_PARKING);
        actionHandler.add(MOVE, CENTER_PARKING);
//        telemetry.addLine("queuing actions");
//        telemetry.addLine(actionHandler.getTotalActions() + " total actions");


        robot.getAutoTelemetry();

        actionHandler.init();
        waitForStart();
        //actionHandler.findAndSetZone();

        while (opModeIsActive()){
            actionHandler.update();
            //robot.getAutoTelemetry();
        }
    }
}