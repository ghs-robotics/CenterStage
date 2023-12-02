package org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.INTAKE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.PLACE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.cv.Camera.SPIKE_ZONE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.ParamHandler;

public class AutoPresets {
    private static AutoActionHandler routeA;
    private static AutoActionHandler beginning;

    static double[] leftSpikePos = {1, 2, 0.0};
    static double[] centerSpikePos = {1, 2, 0.0};
    static double[] rightSpikePos = {1, 2, 0.0};

    static double[] leftBackDropPos = {1, 2, 0.0};
    static double[] centerBackDropPos = {1, 2, 0.0};
    static double[] rightBackDropPos = {1, 2, 0.0};

    static int pixelStackX = 10;
    static int pixelStackY = 10;

    public static AutoActionHandler getRouteA(Robot r, Telemetry t){
        routeA = new AutoActionHandler(r, t);
        routeA.add(INTAKE, new ParamHandler(INTAKE, 0));
        routeA.add(LIFT, new ParamHandler(LIFT, 1));
        routeA.add(EXTEND);
        routeA.add(DROP);
        routeA.add(RETRACT);
        routeA.add(LIFT, new ParamHandler(LIFT, 0));
        return routeA;
    }

    public static AutoActionHandler getBeginningNearBackDrop(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);
        double[] spikePos = new double[3];
        double[] backDropPos = new double[3];

        if (SPIKE_ZONE == 1){
            spikePos = leftSpikePos;
            backDropPos = leftBackDropPos;
        } else if (SPIKE_ZONE == 2) {
            spikePos = centerSpikePos;
            backDropPos = centerBackDropPos;
        }else{
            spikePos = rightSpikePos;
            backDropPos = rightBackDropPos;
        }
        beginning.add(MOVE, new ParamHandler((int) spikePos[0], 0, 0.0));
        beginning.add(MOVE, new ParamHandler(spikePos));
        beginning.add(PLACE);
        beginning.add(MOVE, new ParamHandler((int) backDropPos[0], (int) spikePos[1], 0.0));
        beginning.add(MOVE, new ParamHandler(backDropPos));

        return beginning;
    }

    public static AutoActionHandler getBeginningNearPixels(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);

        double[] spikePos = new double[3];
        double[] backDropPos = new double[3];

        if (SPIKE_ZONE == 1){
            spikePos = leftSpikePos;
            backDropPos = leftBackDropPos;
        } else if (SPIKE_ZONE == 2) {
            spikePos = centerSpikePos;
            backDropPos = centerBackDropPos;
        }else{
            spikePos = rightSpikePos;
            backDropPos = rightBackDropPos;
        }

        beginning.add(MOVE, new ParamHandler((int) spikePos[0], 0, 0.0));
        beginning.add(MOVE, new ParamHandler(spikePos));
        beginning.add(PLACE);
        beginning.add(MOVE, new ParamHandler(pixelStackX, (int) spikePos[1], 0.0));
        beginning.add(MOVE, new ParamHandler(pixelStackX, pixelStackY, 0.0));
        beginning.add(INTAKE, new ParamHandler(INTAKE, 5));
        beginning.add(MOVE, new ParamHandler((int) backDropPos[0], pixelStackY, 0.0));
        beginning.add(MOVE, new ParamHandler(backDropPos));
        beginning.add(DELIVER);

        return beginning;
    }

}
