package org.firstinspires.ftc.teamcode.control.roadrunner.messages;

public final class BallDriveCommandMessage {
    public long timestamp;
    public double voltage;
    public double left;
    public double right;
    public double back;

    public BallDriveCommandMessage(double voltage, double left, double right, double back) {
        this.timestamp = System.nanoTime();
        this.voltage = voltage;
        this.left = left;
        this.right = right;
        this.back = back;
    }
}
