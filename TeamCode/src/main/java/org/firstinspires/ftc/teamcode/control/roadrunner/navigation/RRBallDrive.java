package org.firstinspires.ftc.teamcode.control.roadrunner.navigation;

public class RRBallDrive {


////    public class DriveLocalizer extends Localizer {
////        public final Encoder leftFront, leftBack, rightBack, rightFront;
////
////        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
////        private Rotation2d lastHeading;
////
////        public DriveLocalizer() {
////            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
////            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
////            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
////            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));
////
////            // TODO: reverse encoders if needed
////            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
////
////            lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
////            lastLeftBackPos = leftBack.getPositionAndVelocity().position;
////            lastRightBackPos = rightBack.getPositionAndVelocity().position;
////            lastRightFrontPos = rightFront.getPositionAndVelocity().position;
////
////            lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
////        }
////
////        @Override
////        public Twist2dDual<Time> update() {
////            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
////            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
////            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
////            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();
////
////            FlightRecorder.write("MECANUM_ENCODERS", new MecanumEncodersMessage(
////                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel));
////
////            Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
////            double headingDelta = heading.minus(lastHeading);
////
////            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
////                    new DualNum<Time>(new double[]{
////                            (leftFrontPosVel.position - lastLeftFrontPos),
////                            leftFrontPosVel.velocity,
////                    }).times(PARAMS.inPerTick),
////                    new DualNum<Time>(new double[]{
////                            (leftBackPosVel.position - lastLeftBackPos),
////                            leftBackPosVel.velocity,
////                    }).times(PARAMS.inPerTick),
////                    new DualNum<Time>(new double[]{
////                            (rightBackPosVel.position - lastRightBackPos),
////                            rightBackPosVel.velocity,
////                    }).times(PARAMS.inPerTick),
////                    new DualNum<Time>(new double[]{
////                            (rightFrontPosVel.position - lastRightFrontPos),
////                            rightFrontPosVel.velocity,
////                    }).times(PARAMS.inPerTick)
////            ));
////
////            lastLeftFrontPos = leftFrontPosVel.position;
////            lastLeftBackPos = leftBackPosVel.position;
////            lastRightBackPos = rightBackPosVel.position;
////            lastRightFrontPos = rightFrontPosVel.position;
////
////            lastHeading = heading;
////
////            return new Twist2dDual<>(
////                    twist.line,
////                    DualNum.cons(headingDelta, twist.angle.drop(1))
////            );
////        }
////    }
//package org.firstinspires.ftc.teamcode.control.roadrunner.navigation;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.*;
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.DualNum;
//import com.acmerobotics.roadrunner.HolonomicController;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.MotorFeedforward;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Pose2dDual;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.TimeTrajectory;
//import com.acmerobotics.roadrunner.TimeTurn;
//import com.acmerobotics.roadrunner.TurnConstraints;
//import com.acmerobotics.roadrunner.Twist2dDual;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
//import com.acmerobotics.roadrunner.ftc.FlightRecorder;
//import com.acmerobotics.roadrunner.ftc.LynxFirmware;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.BallDriveCommandMessage;
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.DriveCommandMessage;
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.PoseMessage;
//
//import java.lang.Math;
//import java.util.Arrays;
//import java.util.LinkedList;
//import java.util.List;
//
//@Config
//public class BallDrive {
//    public static Localizer.Params PARAMS = new Localizer.Params();
//
//    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
//            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
//
//    public final Kinematics kinematics = new Kinematics(
//            PARAMS.mmPerTicks * PARAMS.trackWidthTicks, PARAMS.mmPerTicks / PARAMS.lateralMMPerTick);
//
//    public final VelConstraint defaultVelConstraint =
//            new MinVelConstraint(Arrays.asList(
//                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
//                    new AngularVelConstraint(PARAMS.maxAngVel)
//            ));
//    public final AccelConstraint defaultAccelConstraint =
//            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
//
//    public final DcMotorEx leftDrive, rightDrive, backDrive;
//
//    public final VoltageSensor voltageSensor;
//
//    public final Localizer localizer;
//    public Pose2d pose;
//
//    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
//
//    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
//    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
//    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
//    private final DownsampledWriter ballDriveCommandWriter = new DownsampledWriter("BALL_DRIVE_COMMAND", 50_000_000);
//
//    private boolean metaDriveOn = false;
//
//    public BallDrive(HardwareMap hardwareMap, Pose2d pose) {
//        this.pose = pose;
//
//        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);
//
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
//        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
//        backDrive = hardwareMap.get(DcMotorEx.class, "back");
//
//        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//        backDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        localizer = new Localizer(hardwareMap);
//
//        FlightRecorder.write("BALL_DRIVE_PARAMS", PARAMS);
//    }
//
//    public void calculateDrivePowers(double x, double y, double rot, boolean driveMode) {
//        metaDriveOn = driveMode;
//
//        double driveX = x;
//        double driveY = y;
//        double heading = pose.heading.toDouble();
//
//        if (driveMode) {
//            driveX = x * Math.cos(heading) - y * Math.sin(heading);
//            driveY = y * Math.cos(heading) + x * Math.sin(heading);
//        }
//        calculateDrivePowers(driveX, driveY, rot);
//    }
//
//    public void calculateDrivePowers(double x, double y, double rot) {
//        setMotorPowers(
//                y - rot,
//                y + rot,
//                x
//        );
//    }
//
//    public void calculateDrivePowers(PoseVelocity2d powers) {
//        Kinematics.WheelVelocities wheelVels = new Kinematics(PARAMS.trackWidthTicks).inverse(
//                PoseVelocity2dDual.constant(powers, 1));
//
//        double maxPowerMag = 1;
//        for (DualNum power : wheelVels.all()) {
//            maxPowerMag = Math.max(maxPowerMag, power.value());
//        }
//
//        setMotorPowers(
//                (wheelVels.left.get(0) / maxPowerMag),
//                (wheelVels.right.get(0) / maxPowerMag),
//                (wheelVels.back.get(0) / maxPowerMag));
//    }
//
//    public PoseVelocity2d updatePoseEstimate() {
//        Twist2dDual<Time> twist = localizer.update();
//        pose = pose.plus(twist.value());
//
//        poseHistory.add(pose);
//        while (poseHistory.size() > 100) {
//            poseHistory.removeFirst();
//        }
//
//        estimatedPoseWriter.write(new PoseMessage(pose));
//
//        return twist.velocity().value();
//    }
//
//    public boolean getDriveMode() {
//        return metaDriveOn;
//    }
//
//    public double getX(){
//        return pose.position.x;
//    }
//
//    public double getY(){
//        return pose.position.y;
//    }
//
//    public double getHeading(){
//        return pose.heading.toDouble();
//    }
//
//    private void setMotorPowers(double lp, double rp, double bp) {
//        leftDrive.setPower(lp);
//        rightDrive.setPower(rp);
//        backDrive.setPower(bp);
//    }
//
//    public final class FollowTrajectoryAction implements Action {
//        public final TimeTrajectory timeTrajectory;
//        private double beginTs = -1;
//
//        private final double[] xPoints, yPoints;
//
//        public FollowTrajectoryAction(TimeTrajectory t) {
//            timeTrajectory = t;
//
//            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
//                    0, t.path.length(),
//                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
//            xPoints = new double[disps.size()];
//            yPoints = new double[disps.size()];
//            for (int i = 0; i < disps.size(); i++) {
//                Pose2d p = t.path.get(disps.get(i), 1).value();
//                xPoints[i] = p.position.x;
//                yPoints[i] = p.position.y;
//            }
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket p) {
//            double t;
//            if (beginTs < 0) {
//                beginTs = Actions.now();
//                t = 0;
//            } else {
//                t = Actions.now() - beginTs;
//            }
//
//            if (t >= timeTrajectory.duration) {
//                leftDrive.setPower(0);
//                rightDrive.setPower(0);
//                backDrive.setPower(0);
//
//                return false;
//            }
//
//            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
//            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
//
//            PoseVelocity2d robotVelRobot = updatePoseEstimate();
//
//            PoseVelocity2dDual<Time> command = new HolonomicController(
//                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
//                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
//            )
//                    .compute(txWorldTarget, pose, robotVelRobot);
//            driveCommandWriter.write(new DriveCommandMessage(command));
//
//            Kinematics.WheelVelocities wheelVels = kinematics.inverse(command);
//            double voltage = voltageSensor.getVoltage();
//
//            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
//                    PARAMS.kV / PARAMS.mmPerTicks, PARAMS.kA / PARAMS.mmPerTicks);
//            double leftPower = feedforward.compute(wheelVels.left) / voltage;
//            double rightPower = feedforward.compute(wheelVels.right) / voltage;
//            double backPower = feedforward.compute(wheelVels.back) / voltage;
//            ballDriveCommandWriter.write(new BallDriveCommandMessage(
//                    voltage, leftPower, rightPower, backPower
//            ));
//
//            leftDrive.setPower(leftPower);
//            rightDrive.setPower(rightPower);
//            backDrive.setPower(backPower);
//
//            p.put("x", pose.position.x);
//            p.put("y", pose.position.y);
//            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//
//            Pose2d error = txWorldTarget.value().minusExp(pose);
//            p.put("xError", error.position.x);
//            p.put("yError", error.position.y);
//            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
//
//            // only draw when active; only one drive action should be active at a time
//            Canvas c = p.fieldOverlay();
//            drawPoseHistory(c);
//
//            c.setStroke("#4CAF50");
//            drawRobot(c, txWorldTarget.value());
//
//            c.setStroke("#3F51B5");
//            drawRobot(c, pose);
//
//            c.setStroke("#4CAF50FF");
//            c.setStrokeWidth(1);
//            c.strokePolyline(xPoints, yPoints);
//
//            return true;
//        }
//
//        @Override
//        public void preview(Canvas c) {
//            c.setStroke("#4CAF507A");
//            c.setStrokeWidth(1);
//            c.strokePolyline(xPoints, yPoints);
//        }
//    }
//
//    public final class TurnAction implements Action {
//        private final TimeTurn turn;
//
//        private double beginTs = -1;
//
//        public TurnAction(TimeTurn turn) {
//            this.turn = turn;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket p) {
//            double t;
//            if (beginTs < 0) {
//                beginTs = Actions.now();
//                t = 0;
//            } else {
//                t = Actions.now() - beginTs;
//            }
//
//            if (t >= turn.duration) {
//                leftDrive.setPower(0);
//                rightDrive.setPower(0);
//                backDrive.setPower(0);
//                return false;
//            }
//
//            Pose2dDual<Time> txWorldTarget = turn.get(t);
//            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
//
//            PoseVelocity2d robotVelRobot = updatePoseEstimate();
//
//            PoseVelocity2dDual<Time> command = new HolonomicController(
//                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
//                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
//            )
//                    .compute(txWorldTarget, pose, robotVelRobot);
//            driveCommandWriter.write(new DriveCommandMessage(command));
//
//            Kinematics.WheelVelocities wheelVels = kinematics.inverse(command);
//            double voltage = voltageSensor.getVoltage();
//            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
//                    PARAMS.kV / PARAMS.mmPerTicks, PARAMS.kA / PARAMS.mmPerTicks);
//            double leftPower = feedforward.compute(wheelVels.left) / voltage;
//            double rightPower = feedforward.compute(wheelVels.right) / voltage;
//            double backPower = feedforward.compute(wheelVels.back) / voltage;
//            ballDriveCommandWriter.write(new BallDriveCommandMessage(
//                    voltage, leftPower, rightPower, backPower
//            ));
//
//            leftDrive.setPower(feedforward.compute(wheelVels.left) / voltage);
//            rightDrive.setPower(feedforward.compute(wheelVels.right) / voltage);
//            backDrive.setPower(feedforward.compute(wheelVels.back) / voltage);
//
//            Canvas c = p.fieldOverlay();
//            drawPoseHistory(c);
//
//            c.setStroke("#4CAF50");
//            drawRobot(c, txWorldTarget.value());
//
//            c.setStroke("#3F51B5");
//            drawRobot(c, pose);
//
//            c.setStroke("#7C4DFFFF");
//            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
//
//            return true;
//        }
//
//        @Override
//        public void preview(Canvas c) {
//            c.setStroke("#7C4DFF7A");
//            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
//        }
//    }
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
//
//}
//    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
//        return new TrajectoryActionBuilder(
//                new TurnAction(new TimeTurn(beginPose, 0, defaultTurnConstraints)),
//                FollowTrajectoryAction::new,
//                beginPose, 1e-6, 0.0,
//                defaultTurnConstraints,
//                defaultVelConstraint, defaultAccelConstraint,
//                0.25, 0.1
//        );
//    }

}
