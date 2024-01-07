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

public class AutoPresets {
    private static AutoActionHandler routeA;
    private static AutoActionHandler beginning;

    static double[] leftSpikePos = {788, 45, 0.0};
    static double[] centerSpikePos = {900, -131, 0.0};
    static double[] rightSpikePos = {755, -530, 0.0};

    static double[] leftBackDropPos = {744, -880, 0.0};
    static double[] centerBackDropPos = {580, -875, 0.0};
    static double[] rightBackDropPos = {470, -860, 0.0};

//    static int pixelStackX = 3 * TICKS_PER_TILE_X + 20;
//    static int pixelStackY = (int) (1.2 * TICKS_PER_TILE_Y);

    public static AutoActionHandler getRouteA(Robot r, Telemetry t){
        routeA = new AutoActionHandler(r, t);
//        routeA.add(MOVE, 20, (int) (-2.3 * TICKS_PER_TILE_Y), 0.0);
        routeA.add(LIFT, 100);
        routeA.add(EXTEND);
        routeA.add(DROP);
        routeA.add(RETRACT);
        routeA.add(LIFT, 0.0);
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
        beginning.add(MOVE, (int) spikePos[0], 0, 0.0);
//        beginning.add(MOVE, (spikePos));
        beginning.add(PLACE);
        beginning.add(MOVE,(int) backDropPos[0], (int) spikePos[1], 0.0);
//        beginning.add(MOVE, backDropPos));
        beginning.add(DELIVER);

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
//        backDropPos[1] -= 2 * TICKS_PER_TILE_Y;
//
//        beginning.add(MOVE,(int) spikePos[0], 0, 0.0);
//        beginning.add(MOVE, (spikePos));
//        beginning.add(PLACE);
//        beginning.add(MOVE, pixelStackX, (int) spikePos[1], 0.0);
//        beginning.add(MOVE, pixelStackX, pixelStackY, 0.0);
//        beginning.add(INTAKE,  5);
//        beginning.add(MOVE, (int) backDropPos[0], pixelStackY, 0.0);
        beginning.add(MOVE, backDropPos);
        beginning.add(DELIVER);

        return beginning;
    }

}
