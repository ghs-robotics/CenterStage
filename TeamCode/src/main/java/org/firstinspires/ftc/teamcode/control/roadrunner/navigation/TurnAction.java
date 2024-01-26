//package org.firstinspires.ftc.teamcode.control.roadrunner.navigation;
//
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.drawPoseHistory;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.drawRobot;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.kinematics;
//import static org.firstinspires.ftc.teamcode.control.roadrunner.navigation.BallDrive.pose;
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
//import com.acmerobotics.roadrunner.Pose2dDual;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.PoseVelocity2dDual;
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.TimeTurn;
//
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.BallDriveCommandMessage;
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.DriveCommandMessage;
//import org.firstinspires.ftc.teamcode.control.roadrunner.messages.PoseMessage;
//
//public class TurnAction implements Action {
//    private final TimeTurn turn;
//
//    private double beginTs = -1;
//
//    private BallDrive drive;
//
//    public TurnAction(TimeTurn turn) {
//        this.turn = turn;
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
//        if (t >= turn.duration) {
//            drive.setMotorPowers(0, 0, 0);
//            return false;
//        }
//
//        Pose2dDual<Time> txWorldTarget = turn.get(t);
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
//        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
//                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
//        double leftPower = feedforward.compute(wheelVels.left) / voltage;
//        double rightPower = feedforward.compute(wheelVels.right) / voltage;
//        double backPower = feedforward.compute(wheelVels.back) / voltage;
//        PARAMS.ballDriveCommandWriter.write(new BallDriveCommandMessage(
//                voltage, leftPower, rightPower, backPower
//        ));
//
//
//        drive.setMotorPowers(leftPower, rightPower, backPower);
//
//        Canvas c = p.fieldOverlay();
//        drawPoseHistory(c);
//
//        c.setStroke("#4CAF50");
//        drawRobot(c, txWorldTarget.value());
//
//        c.setStroke("#3F51B5");
//        drawRobot(c, pose);
//
//        c.setStroke("#7C4DFFFF");
//        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
//
//        return true;
//    }
//}
