package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.input.Controller;

public class TeleOpProfile {
    Robot robot;
    Controller gp1;
    Controller gp2;
    Telemetry telemetry;

    double driveX;

    public TeleOpProfile (Robot robot, Controller gp1, Controller gp2, Telemetry telemetry) {
        this.robot = robot;
        this.gp1 = gp1;
        this.gp2 = gp2;
        this.telemetry = telemetry;

        // map keys to vars
        driveX = gp1.left_stick_x;



        // map keys to funs
//        robot.drive.calculateDrivePowers(driveX, );
    }
}
