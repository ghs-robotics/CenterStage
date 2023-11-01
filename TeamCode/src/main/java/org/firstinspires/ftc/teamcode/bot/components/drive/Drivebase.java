package org.firstinspires.ftc.teamcode.bot.components.drive;

public interface Drivebase {

    void calculateDrivePowers(double x, double y, double rot, boolean meta);
    void calculateDrivePowers(double x, double y, double rot);
    int[] getEncoderTicks();
    void resetEncoders();
}
