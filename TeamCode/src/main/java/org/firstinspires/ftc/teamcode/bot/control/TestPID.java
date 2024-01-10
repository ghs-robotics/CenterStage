package org.firstinspires.ftc.teamcode.bot.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TestPID {
    private double integralSum = 0;
    private final double kp = 0.3;
    private final double ki = 0;
    private final double kd = 0;
    private double lastError = 0;
    ElapsedTime timer;

    public TestPID () {
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
