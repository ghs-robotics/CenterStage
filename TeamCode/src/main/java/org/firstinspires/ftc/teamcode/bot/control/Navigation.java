package org.firstinspires.ftc.teamcode.bot.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;

@Config
public final class Navigation {
    public static class Params {
        public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;
    public final Gyro gyro;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;


    private double lastRawHeadingVel, headingVelOffset;
    private Telemetry telemetry;

    private final double MM_PER_TICK = (35 * Math.PI) / (8192);

    public static final int TICKS_PER_TILE = 600;

    private double x;
    private double y;
    private double heading;

    Drivebase drive;

    public Navigation(Drivebase drive, Gyro imu, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;

        par = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[0]));
        perp = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[1]));
        this.gyro = imu;

        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
        lastHeading = Rotation2d.exp(gyro.getHeading());


        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
    private double getHeadingVelocity() {
        double rawHeadingVel = gyro.getHeadingVelocity();
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        return headingVelOffset + rawHeadingVel;
    }

    public void update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
        Rotation2d heading = Rotation2d.exp(gyro.getHeading());

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        double headingVel = getHeadingVelocity();

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(MM_PER_TICK),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(MM_PER_TICK)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        this.x += twist.line.y.value();
        this.y += twist.line.x.value();
        this.heading += twist.angle.value();
    }

    // code i wrote
    /**
     * @param x target x position
     * @param y target y position
     * @param heading target heading in degrees
     * @return whether or not the robot is at target position and facing target direction
     */
    public boolean runToPosition(double x, double y, double heading){
        update();

        double xError = x - this.x;
        double yError = this.y - y;
        double counterClockError = (this.heading - (Math.toRadians(heading))) % (2 * Math.PI);
        double clockwiseError = ((Math.toRadians(heading)) - this.heading) % (2 * Math.PI);

        double rotPow = 0;


        // todo replace this with PID
        if (clockwiseError >= counterClockError){
            rotPow = clockwiseError / 10.0;
        } else if (Math.abs(counterClockError - clockwiseError) > Math.toRadians(2)){
            rotPow = -counterClockError /10.0;
        }

        if (Math.abs(xError) > 10) {
            drive.calculateDrivePowers(increasePower(xError / 200.0), 0, 0, true);
        } else if (Math.abs(yError) > 10) {
            drive.calculateDrivePowers(0, increasePower( yError / 200.0), 0, true);
        } else if (rotPow != 0)
            drive.calculateDrivePowers(0,0, rotPow, true);


        telemetry.addLine("y = " + this.y);
        telemetry.addLine("x = " + this.x);
        telemetry.addLine("heading = " + this.heading);
        return Math.abs(xError) < 10 && Math.abs(yError) < 10;

    }

    private double increasePower(double power){
        if (Math.abs(power) < 0.05){
            int sign = (int) (power / Math.abs(power));

            if (sign < 0)
                return -0.1;
            else
                return 0.1;
        }

        return power;
    }

    public double getX(){
        return x;
    }

    /**
     * @return y coordinate
     */
    public double getY(){
        return y;
    }

    /**
     * @return heading of the robot in only radians
     */
    public double getGyroHeading(){
        return heading;
    }

    public void resetNav(){
        drive.resetEncoders();
        gyro.resetHeading();

        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }
}
//package org.firstinspires.ftc.teamcode.bot.control;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.bot.components.Gyro;
//import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;
//
//public class Navigation {
//    // axis based on the robot's starting position
//    private double x;
//    private double y;
//    private double gyroHeading;
//
//    private int yEncoder;
//    private int xEncoder;
//
//    private int deltaX;
//    private int deltaY;
//    private double deltaHeading;
//
//    private final int TICKS_PER_REV = 8192;
//    private final double WHEEL_RAD = 17.5; // in mm
//
//    private final double[] X_DIS_FROM_CENTER = new double[]{164.109, 48.88, 171.23}; // in mm
//    private final double[] Y_DIS_FROM_CENTER = new double[]{153.275, 60.916, 164.94}; // in mm
//
//
//    Telemetry telemetry;
//
//    Drivebase drive;
//    Gyro gyro;
//
//    public Navigation(Drivebase drive, Gyro gyro, Telemetry telemetry){
//        this.drive = drive;
//        this.gyro = gyro;
//
//        this.telemetry = telemetry;
//
//        this.x = 0;
//        this.y = 0;
//
//        updatePosition();
//    }
//
//    public void updatePosition(){
//        updateEncoders();
//
//        this.gyroHeading = gyro.getHeading();
//        this.deltaHeading = gyro.getDeltaHeading();
//
//        // update x position
//        this.x = 2 * (xEncoder / gyroHeading + 12757.38) * (Math.sin(gyroHeading / 2));
////        this.x = xEncoder;
//
//        // update y position
//        this.y = 2 * (yEncoder / gyroHeading + 4538.41) * (Math.sin(gyroHeading / 2));
////        this.y = yEncoder;
//
//    }
//
//    public int runToPosition(double x, double y, double heading, boolean xFirst, int cycle){
//        boolean done;
//
//        if (xFirst)
//            done = runToPosition(x, 0, 0);
//        else if (cycle == 0)
//            done = runToPosition(0, 0, heading);
//        else
//            done = runToPosition(0, y, 0);
//
//
//        if (done)
//            return cycle--;
//        else
//            return cycle;
//    }
//
//    /**
//     * @param x target x position
//     * @param y target y position
//     * @param heading target heading in degrees
//     * @return whether or not the robot is at target position and facing target direction
//     */
//    public boolean runToPosition(double x, double y, double heading){
//        updatePosition();
//
//        double xDiff = x - this.x;
//        double yDiff = this.y - y;
//        double rotDiffCounterClock = (this.gyroHeading - (Math.toRadians(heading))) % (2 * Math.PI);
//        double rotDiffClock = ((Math.toRadians(heading)) - this.gyroHeading) % (2 * Math.PI);
//
//        double xPow = 0;
//        double yPow = 0;
//        double rotPow = 0;
//
//
//        if (Math.abs(xDiff) > TICKS_PER_TILE / 25.0) {
//            xPow = xDiff / 10.0;
//        } else if (x == 0) {
//            xPow = 0;
//        }
//
//        if (Math.abs(yDiff) > TICKS_PER_TILE / 25.0) {
//            yPow = yDiff / 10.0;
//        } else if (y == 0) {
//            yPow = 0;
//        }
//
//        if (rotDiffClock >= rotDiffCounterClock){
//            rotPow = rotDiffClock / 10.0;
//        } else if (Math.abs(rotDiffCounterClock - rotDiffClock) > Math.toRadians(2)){
//            rotPow = rotDiffCounterClock /10.0;
//        } else if (heading == 0)
//            rotPow = 0;
//
//        drive.calculateDrivePowers(xPow , yPow, rotPow, true);
//        telemetry.addLine("y = " + this.y);
//        telemetry.addLine("x = " + this.x);
//        telemetry.addLine("heading = " + this.gyroHeading);
//        telemetry.addLine(String.valueOf(xPow + yPow + rotPow == 0));
//        return xPow + yPow + rotPow < 0.1;
//
//    }
//
//    /**
//     * Helper function that updates the encoder values every cycle
//     */
//    private void updateEncoders() {
//        deltaX = xEncoder + drive.getEncoderTicks()[1];
//        deltaY = yEncoder + drive.getEncoderTicks()[0];
//
//        xEncoder = -drive.getEncoderTicks()[1];
//        yEncoder = -drive.getEncoderTicks()[0];
//
//
//    }
//
//    /**
//     * @return x coordinate
//     */
//    public double getX(){
//        return x;
//    }
//
//    /**
//     * @return y coordinate
//     */
//    public double getY(){
//        return y;
//    }
//
//    /**
//     * @return heading of the robot in only radians
//     */
//    public double getGyroHeading(){
//        return gyroHeading;
//    }
//}
