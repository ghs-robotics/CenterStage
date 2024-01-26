//package org.firstinspires.ftc.teamcode.control.roadrunner.navigation
//
//import com.acmerobotics.roadrunner.Arclength
//import com.acmerobotics.roadrunner.DualNum
//import com.acmerobotics.roadrunner.MecanumKinematics
//import com.acmerobotics.roadrunner.Pose2dDual
//import com.acmerobotics.roadrunner.PosePath
//import com.acmerobotics.roadrunner.PoseVelocity2dDual
//import com.acmerobotics.roadrunner.Twist2dDual
//import com.acmerobotics.roadrunner.Vector2dDual
//import com.acmerobotics.roadrunner.VelConstraint
//import kotlin.math.abs
//
//data class BallDriveKinematics @JvmOverloads constructor(
//        @JvmField
//        val trackWidth: Double,
//        @JvmField
//        val lateralMultiplier: Double = 1.0
//) {
//    /**
//     * @param[wheelbase] distance between wheels on the same side; see the diagram in [MecanumKinematics]
//     */
//    constructor(
//            trackWidth: Double,
//            wheelbase: Double,
//            lateralMultiplier: Double = 1.0
//    ) : this((trackWidth + wheelbase) / 2, lateralMultiplier)
//
//    data class WheelIncrements<Param>(
//            @JvmField
//            val left: DualNum<Param>,
//            @JvmField
//            val right: DualNum<Param>,
//            @JvmField
//            val back: DualNum<Param>,
//    )
//
//    //reminder: x and y is flipped with roadrunner
//    fun <Param> forward(w: WheelIncrements<Param>) = Twist2dDual(
//            Vector2dDual(
//                    (w.left + w.right ) * (0.5 / lateralMultiplier),
//                    (w.back)
//
//            ),
//            // todo find the math for calculating 3 wheel heading and sub in
//            (w.left + w.right) / (2 * trackWidth),
//    )
//
//    data class WheelVelocities<Param>(
//            @JvmField
//            val left: DualNum<Param>,
//            @JvmField
//            val right: DualNum<Param>,
//            @JvmField
//            val back: DualNum<Param>,
//    ) {
//        fun all() = listOf(left, right, back)
//    }
//
//    //reminder: x and y is flipped with roadrunner
//    // todo find out what this does in regular roadrunner and rewrite the math for bd.
//    fun <Param> inverse(t: PoseVelocity2dDual<Param>) = WheelVelocities(
//            t.linearVel.x - t.angVel * 0.5 * trackWidth,
//            t.linearVel.x + t.angVel * 0.5 * trackWidth,
//            t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
//    )
//
//    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
//        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
//            val txRobotWorld = robotPose.value().inverse()
//            val robotVelWorld = robotPose.velocity().value()
//            val robotVelRobot = txRobotWorld * robotVelWorld
//            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
//                    .all()
//                    .minOf { abs(maxWheelVel / it.value()) }
//        }
//    }
//}
