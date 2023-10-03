package org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets;

import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions;

import java.util.ArrayList;

public class Routes {
    private static ArrayList<AutoActionHandler> routeA;

    public static ArrayList<AutoActionHandler> getRouteA(){
        routeA = new ArrayList<>();

        return routeA;
    }
}
