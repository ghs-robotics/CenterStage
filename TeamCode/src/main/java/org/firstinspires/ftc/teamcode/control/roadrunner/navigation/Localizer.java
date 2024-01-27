package org.firstinspires.ftc.teamcode.control.roadrunner.navigation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Localizer {
    public static class Params {
        public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)


        final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
        final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
        final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
        final DownsampledWriter ballDriveCommandWriter = new DownsampledWriter("BALL_DRIVE_COMMAND", 50_000_000);

        // drive model parameters
        public double mmPerTicks = (35 * Math.PI) / 8192;;
        public double lateralMMPerTick = mmPerTicks;
        public double trackWidthTicks = 306.11 / mmPerTicks;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn

    }

    public static Params PARAMS = new Params();

    public final Encoder parLeft, parRight, perp;

    public final double mmPerTicks = (35 * Math.PI) / 8192;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public Localizer(HardwareMap hardwareMap) {
        parLeft = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "left")));
        parRight = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "right")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "back")));

        lastPar0Pos = parLeft.getPositionAndVelocity().position;
        lastPar1Pos = parRight.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

        parLeft.setDirection(DcMotor.Direction.REVERSE);
        parRight.setDirection(DcMotor.Direction.FORWARD);
        perp.setDirection(DcMotor.Direction.REVERSE);

       FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = parLeft.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = parRight.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(mmPerTicks),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(mmPerTicks)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

//        double ds = (deltaR + deltaL) * .5;
//
//        double dx1 = ds * Math.cos(deltaH / 2);
//        double dy1 = ds * Math.sin(deltaH / 2);
//
//        double dx2 = deltaB * Math.sin(deltaH / 2);
//        double dy2 = deltaB * Math.cos(deltaH / 2);
//
//        double dy = dx1 * Math.cos(heading) + dx2 * Math.sin(heading);
//        double dx = - dy1 * Math.sin(heading) + dy2 * Math.cos(heading);
//
//        this.y = this.y + MM_PER_TICK * dy;
//        this.x = this.x + MM_PER_TICK * dx;
        return twist;
    }
}