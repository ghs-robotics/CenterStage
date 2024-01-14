package org.firstinspires.ftc.teamcode.bot.control.auto_execution.presets;

import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.DROP;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.EXTEND;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.LIFT;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.PLACE_PIXEL;
import static org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActions.RETRACT;
import static org.firstinspires.ftc.teamcode.cv.Camera.SPIKE_ZONE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.bot.control.auto_execution.AutoActionHandler;

public class AutoPresets {
    private static AutoActionHandler routeA;
    private static AutoActionHandler beginning;

    public static double[] leftSpikePos = {-660, 0, 0.0};
    public static double[] centerSpikePos = {-900, 0, 0.0};
    public static double[] rightSpikePos = {-660, 490, 0.0};

    public static double[] leftBackDropPos = {744, -880, 0.0};
    public static double[] centerBackDropPos = {-660, -690, 0.0};
    public static double[] rightBackDropPos = {470, -860, 0.0};

//    static int pixelStackX = 3 * TICKS_PER_TILE_X + 20;
//    static int pixelStackY = (int) (1.2 * TICKS_PER_TILE_Y);


    public static AutoActionHandler getLeftSpikePath(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);
        beginning.add(MOVE, -660, 0, 0.0);
        beginning.add(PLACE_PIXEL);
        //backboard
        beginning.add(MOVE, -660, -690, 0.0);

        return beginning;
    }

    public static AutoActionHandler getCenterSpikePath(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);
        //center spike
        beginning.add(MOVE, -900, 0, 0.0);
        beginning.add(PLACE_PIXEL);
        //backboard
        beginning.add(MOVE, -660, -690, 0.0);


        return beginning;
    }

    public static AutoActionHandler getRightSpikePath(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);

        // right
        beginning.add(MOVE, -660, 490, 0.0);
        beginning.add(PLACE_PIXEL);
        //backboard
        beginning.add(MOVE, -660, -690, 0.0);


        return beginning;
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
        beginning.add(PLACE_PIXEL);
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
