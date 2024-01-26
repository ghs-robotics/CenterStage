//package org.firstinspires.ftc.teamcode.control.roadrunner.navigation;
//
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.drawPoseHistory;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.drawRobot;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.kinematics;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.pose;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.Localizer.*;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.Localizer.PARAMS;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Actions;
//import com.acmerobotics.roadrunner.HolonomicController;
//import com.acmerobotics.roadrunner.MotorFeedforward;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Pose2dDual;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.PoseVelocity2dDual;
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.TimeTrajectory;
//import com.acmerobotics.roadrunner.TimeTurn;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Twist2dDual;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.BallDriveCommandMessage;
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.DriveCommandMessage;
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.PoseMessage;
//
//import java.util.List;
//
//public class FollowTrajectoryAction implements Action {
//    public static Params PARAMS = new Params();
//
//    public final TimeTrajectory timeTrajectory;
//    private double beginTs = -1;
//
//    private final double[] xPoints, yPoints;
//
//    private BallDrive drive;
//
//    public FollowTrajectoryAction(TimeTrajectory t) {
//        timeTrajectory = t;
//
//        this.drive = drive;
//
//        List<Double> disps = com.acmerobotics.roadrunner.Math.range(
//                0, t.path.length(),
//                Math.max(2, (int) Math.ceil(t.path.length() / 2)));
//        xPoints = new double[disps.size()];
//        yPoints = new double[disps.size()];
//        for (int i = 0; i < disps.size(); i++) {
//            Pose2d p = t.path.get(disps.get(i), 1).value();
//            xPoints[i] = p.position.x;
//            yPoints[i] = p.position.y;
//        }
//    }
//
//    public void setDrive(BallDrive d){
//        drive = d;
//    }
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket p) {
//        double t;
//        if (beginTs < 0) {
//            beginTs = Actions.now();
//            t = 0;
//        } else {
//            t = Actions.now() - beginTs;
//        }
//
//        if (t >= timeTrajectory.duration) {
//            drive.setMotorPowers(0, 0, 0);
//
//            return false;
//        }
//
//        Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
//        PARAMS.targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
//
//        PoseVelocity2d robotVelRobot = drive.updatePoseEstimate();
//
//        PoseVelocity2dDual<Time> command = new HolonomicController(
//                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
//                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
//        )
//                .compute(txWorldTarget, pose, robotVelRobot);
//        PARAMS.driveCommandWriter.write(new DriveCommandMessage(command));
//
//        BallDriveKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
//        double voltage = drive.voltageSensor.getVoltage();
//
//        // not mec drive specific
//        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
//                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
//        double leftPower = feedforward.compute(wheelVels.left) / voltage;
//        double rightPower = feedforward.compute(wheelVels.right) / voltage;
//        double backPower = feedforward.compute(wheelVels.back) / voltage;
//        PARAMS.ballDriveCommandWriter.write(new BallDriveCommandMessage(
//                voltage, leftPower, rightPower, backPower
//        ));
//
//        drive.setMotorPowers(leftPower, rightPower, backPower);
//
//        p.put("x", pose.position.x);
//        p.put("y", pose.position.y);
//        p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//
//        Pose2d error = txWorldTarget.value().minusExp(pose);
//        p.put("xError", error.position.x);
//        p.put("yError", error.position.y);
//        p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
//
//        // only draw when active; only one drive action should be active at a time
//        Canvas c = p.fieldOverlay();
//        drawPoseHistory(c);
//
//        c.setStroke("#4CAF50");
//        drawRobot(c, txWorldTarget.value());
//
//        c.setStroke("#3F51B5");
//        drawRobot(c, pose);
//
//        c.setStroke("#4CAF50FF");
//        c.setStrokeWidth(1);
//        c.strokePolyline(xPoints, yPoints);
//
//        return true;
//    }
//
//    @Override
//    public void preview(Canvas c) {
//        c.setStroke("#4CAF507A");
//        c.setStrokeWidth(1);
//        c.strokePolyline(xPoints, yPoints);
//    }
//}
