package org.firstinspires.ftc.teamcode.control.roadrunner.navigation;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.NoSuchElementException;

import kotlin.jvm.JvmField;
import kotlin.jvm.internal.DefaultConstructorMarker;
import kotlin.jvm.internal.Intrinsics;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public final class Kinematics {
    public double trackWidth;
    public double lateralMultiplier = 1.0;


    public Kinematics(double trackWidth, double lateralMultiplier) {
        this.trackWidth = trackWidth;
        this.lateralMultiplier = lateralMultiplier;
    }

    public Kinematics(double trackWidth) {
        this(trackWidth, 0.0, 2);
    }

    public Kinematics(double trackWidth, double wheelbase, double lateralMultiplier) {
        this((trackWidth + wheelbase) / (double)2, lateralMultiplier);
    }


    @NotNull
    public Twist2dDual<Time> forward(@NotNull WheelIncrements w) {
        Intrinsics.checkNotNullParameter(w, "w");
        return new Twist2dDual<Time>(new Vector2dDual<Time>(
                w.left.plus(w.right).times(0.5 / this.lateralMultiplier),
                w.back),
                w.left.plus(w.right).div((double)2));// * this.trackWidth));
    }

    @NotNull
    public WheelVelocities inverse(PoseVelocity2dDual<Time> t) {
        Intrinsics.checkNotNullParameter(t, "t");
        return new WheelVelocities(
                t.linearVel.x.minus(t.angVel.times(0.5).times(this.trackWidth)),
                t.linearVel.x.plus(t.angVel.times(0.5).times(this.trackWidth)),
                t.linearVel.y/* .times(this.lateralMultiplier) */.plus(t.angVel.times(this.trackWidth)));
    }


    @NotNull
    public String toString() {
        return "BallDriveKinematics(trackWidth=" + this.trackWidth + ", lateralMultiplier=" + this.lateralMultiplier + ")";
    }
//    // $FF: synthetic method
//    public Kinematics(double var1, double var3, double var5, int var7, DefaultConstructorMarker var8) {
//        this(var1, var3, var5);
//        if ((var7 & 4) != 0) {
//            var5 = 1.0;
//        }
//    }
//    @NotNull
//    public Kinematics copy(double trackWidth, double lateralMultiplier) {
//        return new Kinematics(trackWidth, lateralMultiplier);
//    }
//    // $FF: synthetic method
//    public static Kinematics copy$default(Kinematics var0, double var1, double var3, int var5, Object var6) {
//        if ((var5 & 1) != 0) {
//            var1 = var0.trackWidth;
//        }
//
//        if ((var5 & 2) != 0) {
//            var3 = var0.lateralMultiplier;
//        }
//
//        return var0.copy(var1, var3);
//    }
//    public int hashCode() {
//        return Double.hashCode(this.trackWidth) * 31 + Double.hashCode(this.lateralMultiplier);
//    }
//
//    public boolean equals(@Nullable Object var1) {
//        if (this != var1) {
//            if (var1 instanceof Kinematics) {
//                Kinematics var2 = (Kinematics)var1;
//                if (Double.compare(this.trackWidth, var2.trackWidth) == 0 && Double.compare(this.lateralMultiplier, var2.lateralMultiplier) == 0) {
//                    return true;
//                }
//            }
//
//            return false;
//        } else {
//            return true;
//        }
//    }
//    @Metadata(
//            mv = {1, 9, 0},
//            k = 1,
//            d1 = {"\u0000(\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0000\n\u0000\n\u0002\u0018\u0002\n\u0002\b\b\n\u0002\u0010\u000b\n\u0002\b\u0002\n\u0002\u0010\b\n\u0000\n\u0002\u0010\u000e\n\u0000\b\u0086\b\u0018\u0000*\u0004\b\u0000\u0010\u00012\u00020\u0002B/\u0012\f\u0010\u0003\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004\u0012\f\u0010\u0005\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004\u0012\f\u0010\u0006\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004¢\u0006\u0002\u0010\u0007J\u000f\u0010\b\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0003J\u000f\u0010\t\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0003J\u000f\u0010\n\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0003J?\u0010\u000b\u001a\b\u0012\u0004\u0012\u00028\u00000\u00002\u000e\b\u0002\u0010\u0003\u001a\b\u0012\u0004\u0012\u00028\u00000\u00042\u000e\b\u0002\u0010\u0005\u001a\b\u0012\u0004\u0012\u00028\u00000\u00042\u000e\b\u0002\u0010\u0006\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0001J\u0013\u0010\f\u001a\u00020\r2\b\u0010\u000e\u001a\u0004\u0018\u00010\u0002HÖ\u0003J\t\u0010\u000f\u001a\u00020\u0010HÖ\u0001J\t\u0010\u0011\u001a\u00020\u0012HÖ\u0001R\u0016\u0010\u0006\u001a\b\u0012\u0004\u0012\u00028\u00000\u00048\u0006X\u0087\u0004¢\u0006\u0002\n\u0000R\u0016\u0010\u0003\u001a\b\u0012\u0004\u0012\u00028\u00000\u00048\u0006X\u0087\u0004¢\u0006\u0002\n\u0000R\u0016\u0010\u0005\u001a\b\u0012\u0004\u0012\u00028\u00000\u00048\u0006X\u0087\u0004¢\u0006\u0002\n\u0000¨\u0006\u0013"},
//            d2 = {"Lorg/firstinspires/ftc/teamcode/control/roadrunner/navigation/BallDriveKinematics$WheelIncrements;", "Param", "", "left", "Lcom/acmerobotics/roadrunner/DualNum;", "right", "back", "(Lcom/acmerobotics/roadrunner/DualNum;Lcom/acmerobotics/roadrunner/DualNum;Lcom/acmerobotics/roadrunner/DualNum;)V", "component1", "component2", "component3", "copy", "equals", "", "other", "hashCode", "", "toString", "", "CenterStage4042.TeamCode.main"}
//    )
    public static final class WheelIncrements {
        @NotNull
        public final DualNum<Time> left;
        @NotNull
        public final DualNum<Time> right;
        @NotNull
        public final DualNum<Time> back;

        public WheelIncrements(@NonNull DualNum<Time> left, @NonNull DualNum<Time> right, @NonNull DualNum<Time> back) {
            super();
//            Intrinsics.checkNotNullParameter(left, "left");
//            Intrinsics.checkNotNullParameter(right, "right");
//            Intrinsics.checkNotNullParameter(back, "back");
            this.left = left;
            this.right = right;
            this.back = back;
        }

        @NotNull
        public String toString() {
            return "WheelIncrements(left=" + this.left + ", right=" + this.right + ", back=" + this.back + ")";
        }
}


    public static final class WheelVelocities {
        @NotNull
        public final DualNum<Time> left;
        @NotNull
        public final DualNum<Time> right;
        @NotNull
        public final DualNum<Time> back;

        @NotNull
        public ArrayList<DualNum<Time>> all() {
            ArrayList<DualNum<Time>> array = new ArrayList<>();
            array.add(this.left);
            array.add(this.right);
            array.add(this.back);
            return array;
        }

        public WheelVelocities(@NonNull DualNum<Time> left, @NonNull DualNum<Time> right, @NonNull DualNum<Time> back) {
            super();
            this.left = left;
            this.right = right;
            this.back = back;
        }
        @NotNull
        public String toString() {
            return "WheelVelocities(left=" + this.left + ", right=" + this.right + ", back=" + this.back + ")";
        }
//
//
//        public boolean equals(@Nullable Object var1) {
//            if (this != var1) {
//                if (var1 instanceof WheelVelocities) {
//                    WheelVelocities var2 = (WheelVelocities)var1;
//                    if (Intrinsics.areEqual(this.left, var2.left) && Intrinsics.areEqual(this.right, var2.right) && Intrinsics.areEqual(this.back, var2.back)) {
//                        return true;
//                    }
//                }
//
//                return false;
//            } else {
//                return true;
//            }
//        }
    }
    public final class WheelVelConstraint implements VelConstraint {
        public final double maxWheelVel;

        public double maxRobotVel(@NotNull Pose2dDual<Arclength> robotPose, @NotNull PosePath path, double s) {
            Pose2d txRobotWorld = robotPose.value().inverse();
            PoseVelocity2d robotVelWorld = robotPose.velocity().value();
            PoseVelocity2d robotVelRobot = txRobotWorld.times(robotVelWorld);
            ArrayList<DualNum<Time>> motors = Kinematics.this.inverse(PoseVelocity2dDual.Companion.constant(robotVelRobot, 1)).all();
            Iterator<DualNum<Time>> var9 = motors.iterator();
            if (!var9.hasNext()) {
                throw new NoSuchElementException();
            } else {
                DualNum<Time> itx = var9.next();
                double var12 = this.maxWheelVel / itx.value();

                double var14;
                for(var14 = Math.abs(var12); var9.hasNext(); var14 = Math.min(var14, var12)) {
                    DualNum<Time> it = var9.next();
                    double var18 = this.maxWheelVel / it.value();
                    var12 = Math.abs(var18);
                }

                return var14;
            }
        }

        public WheelVelConstraint(double maxWheelVel) {
            this.maxWheelVel = maxWheelVel;
        }
    }
}

//            Intrinsics.checkNotNullParameter(left, "left");
//            Intrinsics.checkNotNullParameter(right, "right");
//            Intrinsics.checkNotNullParameter(back, "back");
//            return new WheelIncrements(left, right, back);
//        }


//
//    // $FF: synthetic method
//    public Kinematics(double var1, double var3, int var5) {
//        this(var1, var3);
//        if ((var5 & 2) != 0) {
//            var3 = 1.0;
//        }
//
//    }

//
//@Metadata(
//        mv = {1, 9, 0},
//        k = 1,
//        d1 = {"\u0000B\n\u0002\u0018\u0002\n\u0002\u0010\u0000\n\u0000\n\u0002\u0010\u0006\n\u0002\b\b\n\u0002\u0010\u000b\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\b\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u000e\n\u0002\b\u0004\b\u0086\b\u0018\u00002\u00020\u0001:\u0003\u001b\u001c\u001dB!\b\u0016\u0012\u0006\u0010\u0002\u001a\u00020\u0003\u0012\u0006\u0010\u0004\u001a\u00020\u0003\u0012\b\b\u0002\u0010\u0005\u001a\u00020\u0003¢\u0006\u0002\u0010\u0006B\u0019\b\u0007\u0012\u0006\u0010\u0002\u001a\u00020\u0003\u0012\b\b\u0002\u0010\u0005\u001a\u00020\u0003¢\u0006\u0002\u0010\u0007J\t\u0010\b\u001a\u00020\u0003HÆ\u0003J\t\u0010\t\u001a\u00020\u0003HÆ\u0003J\u001d\u0010\n\u001a\u00020\u00002\b\b\u0002\u0010\u0002\u001a\u00020\u00032\b\b\u0002\u0010\u0005\u001a\u00020\u0003HÆ\u0001J\u0013\u0010\u000b\u001a\u00020\f2\b\u0010\r\u001a\u0004\u0018\u00010\u0001HÖ\u0003J \u0010\u000e\u001a\b\u0012\u0004\u0012\u0002H\u00100\u000f\"\u0004\b\u0000\u0010\u00102\f\u0010\u0011\u001a\b\u0012\u0004\u0012\u0002H\u00100\u0012J\t\u0010\u0013\u001a\u00020\u0014HÖ\u0001J \u0010\u0015\u001a\b\u0012\u0004\u0012\u0002H\u00100\u0016\"\u0004\b\u0000\u0010\u00102\f\u0010\u0017\u001a\b\u0012\u0004\u0012\u0002H\u00100\u0018J\t\u0010\u0019\u001a\u00020\u001aHÖ\u0001R\u0010\u0010\u0005\u001a\u00020\u00038\u0006X\u0087\u0004¢\u0006\u0002\n\u0000R\u0010\u0010\u0002\u001a\u00020\u00038\u0006X\u0087\u0004¢\u0006\u0002\n\u0000¨\u0006\u001e"},
//        d2 = {"Lorg/firstinspires/ftc/teamcode/control/roadrunner/navigation/BallDriveKinematics;", "", "trackWidth", "", "wheelbase", "lateralMultiplier", "(DDD)V", "(DD)V", "component1", "component2", "copy", "equals", "", "other", "forward", "Lcom/acmerobotics/roadrunner/Twist2dDual;", "Param", "w", "Lorg/firstinspires/ftc/teamcode/control/roadrunner/navigation/BallDriveKinematics$WheelIncrements;", "hashCode", "", "inverse", "Lorg/firstinspires/ftc/teamcode/control/roadrunner/navigation/BallDriveKinematics$WheelVelocities;", "t", "Lcom/acmerobotics/roadrunner/PoseVelocity2dDual;", "toString", "", "WheelIncrements", "WheelVelConstraint", "WheelVelocities", "CenterStage4042.TeamCode.main"}
//)

//
//        public boolean equals(@Nullable Object var1) {
//            if (this != var1) {
//                if (var1 instanceof WheelIncrements) {
//                    WheelIncrements var2 = (WheelIncrements)var1;
//                    if (Intrinsics.areEqual(this.left, var2.left) && Intrinsics.areEqual(this.right, var2.right) && Intrinsics.areEqual(this.back, var2.back)) {
//                        return true;
//                    }
//                }
//
//                return false;
//            } else {
//                return true;
//            }
//        }


//        // $FF: synthetic method
//        public static WheelIncrements copy$default(WheelIncrements var0, DualNum var1, DualNum var2, DualNum var3, int var4, Object var5) {
//            if ((var4 & 1) != 0) {
//                var1 = var0.left;
//            }
//
//            if ((var4 & 2) != 0) {
//                var2 = var0.right;
//            }
//
//            if ((var4 & 4) != 0) {
//                var3 = var0.back;
//            }
//
//            return var0.copy(var1, var2, var3);
//        }


//        public int hashCode() {
//            DualNum var10000 = this.left;
//            int var1 = (var10000 != null ? var10000.hashCode() : 0) * 31;
//            DualNum var10001 = this.right;
//            var1 = (var1 + (var10001 != null ? var10001.hashCode() : 0)) * 31;
//            var10001 = this.back;
//            return var1 + (var10001 != null ? var10001.hashCode() : 0);
//        }

//    @Metadata(
//            mv = {1, 9, 0},
//            k = 1,
//            d1 = {"\u00000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0000\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0004\n\u0002\u0010 \n\u0002\b\u0005\n\u0002\u0010\u000b\n\u0002\b\u0002\n\u0002\u0010\b\n\u0000\n\u0002\u0010\u000e\n\u0000\b\u0086\b\u0018\u0000*\u0004\b\u0000\u0010\u00012\u00020\u0002B/\u0012\f\u0010\u0003\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004\u0012\f\u0010\u0005\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004\u0012\f\u0010\u0006\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004¢\u0006\u0002\u0010\u0007J\u0012\u0010\b\u001a\u000e\u0012\n\u0012\b\u0012\u0004\u0012\u00028\u00000\u00040\tJ\u000f\u0010\n\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0003J\u000f\u0010\u000b\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0003J\u000f\u0010\f\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0003J?\u0010\r\u001a\b\u0012\u0004\u0012\u00028\u00000\u00002\u000e\b\u0002\u0010\u0003\u001a\b\u0012\u0004\u0012\u00028\u00000\u00042\u000e\b\u0002\u0010\u0005\u001a\b\u0012\u0004\u0012\u00028\u00000\u00042\u000e\b\u0002\u0010\u0006\u001a\b\u0012\u0004\u0012\u00028\u00000\u0004HÆ\u0001J\u0013\u0010\u000e\u001a\u00020\u000f2\b\u0010\u0010\u001a\u0004\u0018\u00010\u0002HÖ\u0003J\t\u0010\u0011\u001a\u00020\u0012HÖ\u0001J\t\u0010\u0013\u001a\u00020\u0014HÖ\u0001R\u0016\u0010\u0006\u001a\b\u0012\u0004\u0012\u00028\u00000\u00048\u0006X\u0087\u0004¢\u0006\u0002\n\u0000R\u0016\u0010\u0003\u001a\b\u0012\u0004\u0012\u00028\u00000\u00048\u0006X\u0087\u0004¢\u0006\u0002\n\u0000R\u0016\u0010\u0005\u001a\b\u0012\u0004\u0012\u00028\u00000\u00048\u0006X\u0087\u0004¢\u0006\u0002\n\u0000¨\u0006\u0015"},
//            d2 = {"Lorg/firstinspires/ftc/teamcode/control/roadrunner/navigation/BallDriveKinematics$WheelVelocities;", "Param", "", "left", "Lcom/acmerobotics/roadrunner/DualNum;", "right", "back", "(Lcom/acmerobotics/roadrunner/DualNum;Lcom/acmerobotics/roadrunner/DualNum;Lcom/acmerobotics/roadrunner/DualNum;)V", "all", "", "component1", "component2", "component3", "copy", "equals", "", "other", "hashCode", "", "toString", "", "CenterStage4042.TeamCode.main"}
//    )

//        public int hashCode() {
//            DualNum var10000 = this.left;
//            int var1 = (var10000 != null ? var10000.hashCode() : 0) * 31;
//            DualNum var10001 = this.right;
//            var1 = (var1 + (var10001 != null ? var10001.hashCode() : 0)) * 31;
//            var10001 = this.back;
//            return var1 + (var10001 != null ? var10001.hashCode() : 0);
//        }

//
//        @NotNull
//        public final WheelVelocities copy(@NotNull DualNum left, @NotNull DualNum right, @NotNull DualNum back) {
//            Intrinsics.checkNotNullParameter(left, "left");
//            Intrinsics.checkNotNullParameter(right, "right");
//            Intrinsics.checkNotNullParameter(back, "back");
//            return new WheelVelocities(left, right, back);
//        }
//
//        // $FF: synthetic method
//        public static WheelVelocities copy$default(WheelVelocities var0, DualNum var1, DualNum var2, DualNum var3, int var4, Object var5) {
//            if ((var4 & 1) != 0) {
//                var1 = var0.left;
//            }
//
//            if ((var4 & 2) != 0) {
//                var2 = var0.right;
//            }
//
//            if ((var4 & 4) != 0) {
//                var3 = var0.back;
//            }
//
//            return var0.copy(var1, var2, var3);
//        }


//    @Metadata(
//            mv = {1, 9, 0},
//            k = 1,
//            d1 = {"\u0000$\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0006\n\u0002\b\u0003\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\b\u0086\u0004\u0018\u00002\u00020\u0001B\r\u0012\u0006\u0010\u0002\u001a\u00020\u0003¢\u0006\u0002\u0010\u0004J&\u0010\u0005\u001a\u00020\u00032\f\u0010\u0006\u001a\b\u0012\u0004\u0012\u00020\b0\u00072\u0006\u0010\t\u001a\u00020\n2\u0006\u0010\u000b\u001a\u00020\u0003H\u0016R\u0010\u0010\u0002\u001a\u00020\u00038\u0006X\u0087\u0004¢\u0006\u0002\n\u0000¨\u0006\f"},
//            d2 = {"Lorg/firstinspires/ftc/teamcode/control/roadrunner/navigation/BallDriveKinematics$WheelVelConstraint;", "Lcom/acmerobotics/roadrunner/VelConstraint;", "maxWheelVel", "", "(Lorg/firstinspires/ftc/teamcode/control/roadrunner/navigation/BallDriveKinematics;D)V", "maxRobotVel", "robotPose", "Lcom/acmerobotics/roadrunner/Pose2dDual;", "Lcom/acmerobotics/roadrunner/Arclength;", "path", "Lcom/acmerobotics/roadrunner/PosePath;", "s", "CenterStage4042.TeamCode.main"}
//    )
