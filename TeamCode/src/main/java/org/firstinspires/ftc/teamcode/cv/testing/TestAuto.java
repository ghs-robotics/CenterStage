package org.firstinspires.ftc.teamcode.cv.testing;

import static org.firstinspires.ftc.teamcode.bot.control.Navigation.TICKS_PER_TILE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.INTAKE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.WAIT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets.AutoPresets;
import org.firstinspires.ftc.teamcode.cv.Camera;

@Autonomous
public class TestAuto extends LinearOpMode {
    Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {




        telemetry.addLine("queuing actions");

        waitForStart();

        while (opModeIsActive()){
        }

    }
}
