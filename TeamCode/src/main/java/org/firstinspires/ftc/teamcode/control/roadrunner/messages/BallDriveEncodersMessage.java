package org.firstinspires.ftc.teamcode.control.roadrunner.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class BallDriveEncodersMessage {
    public long timestamp;
    public PositionVelocityPair left;
    public PositionVelocityPair right;
    public PositionVelocityPair back;

    public BallDriveEncodersMessage(PositionVelocityPair left, PositionVelocityPair right, PositionVelocityPair back) {
        this.timestamp = System.nanoTime();
        this.left = left;
        this.right = right;
        this.back = back;
    }
}
