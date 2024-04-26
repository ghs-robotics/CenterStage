package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.BACK_ADJUST_Y;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.CENTER_PARKING;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.CORNER_PARKING;
import static org.firstinspires.ftc.teamcode.presets.AutoPresets.basicLong;
import static org.firstinspires.ftc.teamcode.presets.AutoPresets.goToSpikeMark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActionHandler;
@Autonomous
public class AutoBlueLong extends LinearOpMode {
    Robot robot;
    AutoActionHandler actionHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, false);
        actionHandler = new AutoActionHandler(robot, telemetry, true);
        robot.init();

        // create list of actions to run
//        actionHandler.add(RETRACT);
//        actionHandler.add(basicLong(robot, telemetry));
        actionHandler.add(goToSpikeMark(robot, telemetry));

//        actionHandler.add(MOVE, (int) CORNER_PARKING[0], (int) (CORNER_PARKING[1] + BACK_ADJUST_Y), 0.0);
//        actionHandler.add(MOVE, (int) CENTER_PARKING[0], (int) (CENTER_PARKING[1] + BACK_ADJUST_Y), 0.0);



        telemetry.addLine("queuing actions");
        telemetry.addLine(actionHandler.getTotalActions() + " total actions");

        waitForStart();
        //actionHandler.findAndSetZone();
        actionHandler.init();

        while (opModeIsActive()){
            actionHandler.update();
        }
    }
}
