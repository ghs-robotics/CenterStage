//package org.firstinspires.ftc.teamcode.bot.control;
//
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.roadrunner.DualNum;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Rotation2d;
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.Twist2dDual;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.Vector2dDual;
//import com.acmerobotics.roadrunner.ftc.Encoder;
//import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
//import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
//import com.acmerobotics.roadrunner.ftc.RawEncoder;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.bot.components.Gyro;
//import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;
//import org.opencv.core.Mat;
//
//import java.util.LinkedList;
//
//public class Navigation {
//
//    private double kS = 0;
//    private double kV = 0;
//    private double kA = 0;
//
//    public boolean runToPosition(int targetX, int targetY, double heading) {
//        if (Math.abs(targetX - x) < 25)
//            drive.calculateDrivePowers((targetX - x) / 50.0, 0, 0, true);
//        else if (Math.abs(targetY - y) < 30) {
//            drive.calculateDrivePowers(0, -(targetY - y) / 50.0, 0, true);
//
//        } else
//            return true;
//
//        return false;
//    }
//
//    public static class Params {
//        public double parYLeftTicks = 0.0; // y position of the parallel encoder (in tick units)
//        public double parYRightTicks = 1.0; // y position of the parallel encoder (in tick units)
//        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
//    }
//
//    public static Params PARAMS = new Params();
//
//    public final Encoder parLeft, parRight, perp;
//    public final Gyro gyro;
//
//    private int lastLeftParPos, lastRightParPos, lastPerpPos;
//    private Rotation2d lastHeading;
//
//    private Pose2d pose;
//
//
//    private double lastRawHeadingVel, headingVelOffset;
//    private Telemetry telemetry;
//
//    private final double MM_PER_TICK = (35 * Math.PI) / (8192);
//
//    public static final int TICKS_PER_TILE_X = 600;
//    public static final int TICKS_PER_TILE_Y = 600;
//
//    private double x;
//    private double y;
//    private double heading;
//
//    Drivebase drive;
//
//    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
//
//    public Navigation(Drivebase drive, Gyro gyro, Telemetry telemetry) {
//        this.drive = drive;
//        this.telemetry = telemetry;
//
//        this.pose = new Pose2d(0,0, gyro.getHeading());
//
//        parLeft = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[0]));
//        parRight = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[1]));
//        perp = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[2]));
//        this.gyro = gyro;
//
//        lastLeftParPos = parLeft.getPositionAndVelocity().position;
//        lastRightParPos = parLeft.getPositionAndVelocity().position + 1;
//        lastPerpPos = perp.getPositionAndVelocity().position;
//        lastHeading = Rotation2d.exp(gyro.getHeading());
//    }
//
//    public void resetNav() {
//        x = 0;
//        y = 0;
//        heading = 0;
//
//        pose = new Pose2d(0,0,0);
//        for (DcMotorEx motor: drive.getEncoderMotors()) {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//    }
//
//    public void update(){
//        updatePose();
//
////        x = pose.position.x;
////        y = pose.position.y;
//        heading = gyro.getHeading();
//    }
//
//    public PoseVelocity2d updatePose() {
//        Twist2dDual<Time> twist = this.updateTwist();
//
////        pose = pose.plus(twist.value());
////
////        this.x += twist.line.y.value();
////        this.y += twist.line.x.value();
////        this.heading += twist.angle.value();
//
//        poseHistory.add(pose);
//        while (poseHistory.size() > 100) {
//            poseHistory.removeFirst();
//        }
//
//        return twist.velocity().value();
//    }
//
//    private Twist2dDual<Time> updateTwist(){
//        PositionVelocityPair parLeftPosVel = parLeft.getPositionAndVelocity();
//        PositionVelocityPair parRightPosVel = parRight.getPositionAndVelocity();
//        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
//
//        int parLeftPosDelta = parLeftPosVel.position - lastLeftParPos;
//        int parRightPosDelta = parRightPosVel.position - lastRightParPos;
//        int perpPosDelta = perpPosVel.position - lastPerpPos;
//
//        Twist2dDual<Time> twist = new Twist2dDual<>(
//                new Vector2dDual<>(
//                        new DualNum<Time>(new double[] {
//                                (PARAMS.parYLeftTicks * parRightPosDelta - PARAMS.parYRightTicks * parLeftPosDelta)
//                                        / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
//                                (PARAMS.parYLeftTicks * parRightPosVel.velocity - PARAMS.parYRightTicks * parLeftPosVel.velocity)
//                                        / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
//                        }).times(MM_PER_TICK),
//                        new DualNum<Time>(new double[] {
//                                (PARAMS.perpXTicks / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks) *
//                                        (parRightPosDelta - parLeftPosDelta) + perpPosDelta),
//                                (PARAMS.perpXTicks / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks) *
//                                        (parRightPosVel.velocity - parLeftPosVel.velocity) + perpPosVel.velocity),
//                        }).times(MM_PER_TICK)
//                ),
//                new DualNum<>(new double[] {
//                        (parLeftPosDelta - parRightPosDelta) / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
//                        (parLeftPosVel.velocity - parRightPosVel.velocity) / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
//                })
//        );
//
//
//        lastLeftParPos = parLeftPosVel.position;
//        lastRightParPos = parRightPosVel.position;
//        lastPerpPos = perpPosVel.position;
////
//        this.x += twist.line.y.value();
//        this.y += twist.line.x.value();
////        this.heading += twist.angle.value();
//
//        return twist;
//    }
//
//    public double getX() {
//        return x;
//    }
//
//    public double getY() {
//        return y;
//    }
//
//    public double getHeading() {
//        return gyro.getHeading();
//    }
//
//
//    private void drawPoseHistory(Canvas c) {
//        double[] xPoints = new double[poseHistory.size()];
//        double[] yPoints = new double[poseHistory.size()];
//
//        int i = 0;
//        for (Pose2d t : poseHistory) {
//            xPoints[i] = t.position.x;
//            yPoints[i] = t.position.y;
//
//            i++;
//        }
//
//        c.setStrokeWidth(1);
//        c.setStroke("#3F51B5");
//        c.strokePolyline(xPoints, yPoints);
//    }
//
//    private static void drawRobot(Canvas c, Pose2d t) {
//        final double ROBOT_RADIUS = 9;
//
//        c.setStrokeWidth(1);
//        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);
//
//        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
//        Vector2d p1 = t.position.plus(halfv);
//        Vector2d p2 = p1.plus(halfv);
//        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
//    }
//}


package org.firstinspires.ftc.teamcode.bot.control;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.components.Gyro;
import org.firstinspires.ftc.teamcode.bot.components.drive.Drivebase;
import org.opencv.core.Mat;

import java.util.LinkedList;

public class Navigation {

    private double kS = 0;
    private double kV = 0;
    private double kA = 0;

    public boolean runToPosition(int targetX, int targetY, double heading) {
        if (Math.abs(targetX - x) < 25)
            drive.calculateDrivePowers((targetX - x) / 50.0, 0, 0, true);
        else if (Math.abs(targetY - y) < 30) {
            drive.calculateDrivePowers(0, -(targetY - y) / 50.0, 0, true);

        } else
            return true;

        return false;
    }

    public static class Params {
        public double parYLeftTicks = 0.0; // y position of the parallel encoder (in tick units)
        public double parYRightTicks = 1.0; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder parLeft, parRight, perp;
    public final Gyro gyro;

    private int lastLeftParPos, lastRightParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private Pose2d pose;


    private double lastRawHeadingVel, headingVelOffset;
    private Telemetry telemetry;

    private final double MM_PER_TICK = (35 * Math.PI) / (8192);

    public static final int TICKS_PER_TILE_X = 600;
    public static final int TICKS_PER_TILE_Y = 600;

    private double x;
    private double y;
    private double heading;

    Drivebase drive;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public Navigation(Drivebase drive, Gyro gyro, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;

        this.pose = new Pose2d(0,0, gyro.getHeading());

        parLeft = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[0]));
        parRight = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[1]));
        perp = new OverflowEncoder(new RawEncoder(drive.getEncoderMotors()[2]));
        this.gyro = gyro;

        lastLeftParPos = parLeft.getPositionAndVelocity().position;
        lastRightParPos = parLeft.getPositionAndVelocity().position + 1;
        lastPerpPos = perp.getPositionAndVelocity().position;
        lastHeading = Rotation2d.exp(gyro.getHeading());
    }

    public void resetNav() {
        x = 0;
        y = 0;
        heading = 0;

        pose = new Pose2d(0,0,0);
        for (DcMotorEx motor: drive.getEncoderMotors()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void update(){
        updatePose();

//        x = pose.position.x;
//        y = pose.position.y;
        heading = gyro.getHeading();
    }

    public PoseVelocity2d updatePose() {
        Twist2dDual<Time> twist = this.updateTwist();

//        pose = pose.plus(twist.value());
//
//        this.x += twist.line.y.value();
//        this.y += twist.line.x.value();
//        this.heading += twist.angle.value();

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        return twist.velocity().value();
    }

    private Twist2dDual<Time> updateTwist(){
        PositionVelocityPair parLeftPosVel = parLeft.getPositionAndVelocity();
        PositionVelocityPair parRightPosVel = parRight.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int parLeftPosDelta = parLeftPosVel.position - lastLeftParPos;
        int parRightPosDelta = parRightPosVel.position - lastRightParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.parYLeftTicks * parRightPosDelta - PARAMS.parYRightTicks * parLeftPosDelta)
                                        / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
                                (PARAMS.parYLeftTicks * parRightPosVel.velocity - PARAMS.parYRightTicks * parLeftPosVel.velocity)
                                        / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
                        }).times(MM_PER_TICK),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks) *
                                        (parRightPosDelta - parLeftPosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks) *
                                        (parRightPosVel.velocity - parLeftPosVel.velocity) + perpPosVel.velocity),
                        }).times(MM_PER_TICK)
                ),
                new DualNum<>(new double[] {
                        (parLeftPosDelta - parRightPosDelta) / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
                        (parLeftPosVel.velocity - parRightPosVel.velocity) / (PARAMS.parYLeftTicks - PARAMS.parYRightTicks),
                })
        );


        lastLeftParPos = parLeftPosVel.position;
        lastRightParPos = parRightPosVel.position;
        lastPerpPos = perpPosVel.position;
//
        this.x += twist.line.y.value();
        this.y += twist.line.x.value();
//        this.heading += twist.angle.value();

        return twist;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return gyro.getHeading();
    }


    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
