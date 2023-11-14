package org.firstinspires.ftc.teamcode.bot.components.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public interface Drivebase {

    void calculateDrivePowers(double x, double y, double rot, boolean meta);
    void calculateDrivePowers(double x, double y, double rot);
    int[] getEncoderTicks();
    void resetEncoders();
    boolean getDriveMode();

    DcMotorEx[] getEncoderMotors();
}
