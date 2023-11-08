package org.firstinspires.ftc.teamcode.bot.control;

public class PID {
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;

    private double integralSum = 0;
    private double lastError = 0;

    // how to use in auto:
    // PIDControl(target position for x or y, motor.getCurrentPosition(),timer)

    public double PIDControl (double target, double current, double timer) {
        double error = target - current;
        integralSum += error * (timer / 1000);

        double derivative = (error - lastError) * (timer / 1000);
        lastError = error;

        double output = (error * kp) + (derivative * kd) + (integralSum * ki);

        return output;
    }
}