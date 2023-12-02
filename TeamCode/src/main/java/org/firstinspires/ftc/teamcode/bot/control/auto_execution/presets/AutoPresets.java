//package org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets;
//
//import static org.firstinspires.ftc.teamcode.bot.control.Navigation.TICKS_PER_TILE;
//import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DROP;
//import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.EXTEND;
//import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.INTAKE;
//import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
//import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
//import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.RETRACT;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.bot.Robot;
//import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
//import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;
//
//public class AutoPresets {
//    private static AutoActionHandler routeA;
//
//    public static AutoActionHandler getRouteA(Robot r, Telemetry t){
//        routeA = new AutoActionHandler(r, t);
//        routeA.add(INTAKE, new ParamHandler(INTAKE, 0));
//        routeA.add(LIFT, new ParamHandler(LIFT, 1));
//        routeA.add(EXTEND);
//        routeA.add(DROP);
//        routeA.add(RETRACT);
//        routeA.add(LIFT, new ParamHandler(LIFT, 0));
//        return routeA;
//    }
//}
