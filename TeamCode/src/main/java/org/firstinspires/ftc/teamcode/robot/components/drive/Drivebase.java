package org.firstinspires.ftc.teamcode.robot.components.drive;

public interface Drivebase {
    void calculateDrivePowers(double x, double y, double rot);
    int[] getEncoderTicks();
}
