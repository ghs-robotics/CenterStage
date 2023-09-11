package org.firstinspires.ftc.teamcode.bot.components.drive;

public interface Drivebase {
    void calculateDrivePowers(double x, double y, double rot);
    int[] getEncoderTicks();
}
