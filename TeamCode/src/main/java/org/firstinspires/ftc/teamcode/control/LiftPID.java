package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftPID {
    private double integralSum = 0;
    private final double kp = 1;
    private final double ki = 0.00162824;
    private final double kd = 0.001674;
    private double lastError = 0;
    ElapsedTime timer;

    public LiftPID() {
    timer = new ElapsedTime();
    }
    public double PID(double reference, double state) {
        double error = reference - state;
        integralSum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * kp) + (derivative * kd) + (integralSum * ki);
    }
}