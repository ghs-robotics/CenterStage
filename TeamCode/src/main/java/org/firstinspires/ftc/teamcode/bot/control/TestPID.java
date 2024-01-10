package org.firstinspires.ftc.teamcode.bot.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TestPID {
    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public double PID (double reference, double state) {
        double error = reference - state;
        integralSum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * kp) * (derivative * kd) * (integralSum * ki);
    }
}
