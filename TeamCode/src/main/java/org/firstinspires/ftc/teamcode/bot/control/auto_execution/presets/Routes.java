package org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets;

import static org.firstinspires.ftc.teamcode.bot.control.Navigation.TICKS_PER_TILE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.RETRACT;

import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;

import java.util.ArrayList;

public class Routes {
    private static ArrayList<AutoActionHandler> routeA;

    public static ArrayList<AutoActionHandler> getRouteA(){
        routeA = new ArrayList<>();
//        routeA.add(MOVE, new ParamHandler((TICKS_PER_TILE), (int) -(TICKS_PER_TILE * 1.3), 0.0));
//        routeA.add(LIFT);
//        routeA.add(EXTEND);
//        routeA.add(DROP);
//        routeA.add(RETRACT);
        return routeA;
    }
}
