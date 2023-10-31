package org.firstinspires.ftc.teamcode.bot.components.drive;

public interface Drivebase {

    void metaDrive(double x, double y, double rot);
    void calculateDrivePowers(double x, double y, double rot);
    int[] getEncoderTicks();
    void resetEncoders();
}
