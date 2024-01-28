package org.firstinspires.ftc.teamcode.control.presets;

import static org.firstinspires.ftc.teamcode.control.cv.Camera.SPIKE_ZONE;
import static org.firstinspires.ftc.teamcode.control.presets.Position.CENTER_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.CENTER_SPIKE_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.LEFT_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.LEFT_SPIKE_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.RIGHT_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.RIGHT_SPIKE_POS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions;

public class AutoPresets {
    private static AutoActionHandler routeA;
    private static AutoActionHandler beginning;

//    static int pixelStackX = 3 * TICKS_PER_TILE_X + 20;
//    static int pixelStackY = (int) (1.2 * TICKS_PER_TILE_Y);


    public static AutoActionHandler getLeftSpikePath(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);
        beginning.add(AutoActions.MOVE, -660, 0, 0.0);
        beginning.add(AutoActions.PLACE_PIXEL);
        //backboard
        beginning.add(AutoActions.MOVE, -660, -690, 0.0);

        return beginning;
    }

    public static AutoActionHandler getCenterSpikePath(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);
        //center spike
        beginning.add(AutoActions.MOVE, -900, 0, 0.0);
        beginning.add(AutoActions.PLACE_PIXEL);
        //backboard
        beginning.add(AutoActions.MOVE, -660, -690, 0.0);


        return beginning;
    }

    public static AutoActionHandler getRightSpikePath(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);

        // right
        beginning.add(AutoActions.MOVE, -660, 490, 0.0);
        beginning.add(AutoActions.PLACE_PIXEL);
        //backboard
        beginning.add(AutoActions.MOVE, -660, -690, 0.0);


        return beginning;
    }

    public static AutoActionHandler parkRed (Robot r, Telemetry t) {
        beginning = new AutoActionHandler(r, t);
        beginning.add(AutoActions.MOVE, 100, 10, 0.0);

    return beginning;
    }

    public static AutoActionHandler parkBlue (Robot r, Telemetry t) {
        beginning = new AutoActionHandler(r, t);
        beginning.add(AutoActions.MOVE, 100, 10, 0.0);

        return beginning;
    }


    public static AutoActionHandler getBeginningNearBackDrop(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);
        double[] spikePos = new double[3];
        double[] backDropPos = new double[3];

        if (SPIKE_ZONE == 1){
            spikePos = LEFT_SPIKE_POS;
            backDropPos = LEFT_BACKDROP_POS;
        } else if (SPIKE_ZONE == 2) {
            spikePos = CENTER_SPIKE_POS;
            backDropPos = CENTER_BACKDROP_POS;
        }else{
            spikePos = RIGHT_SPIKE_POS;
            backDropPos = RIGHT_BACKDROP_POS;
        }
        beginning.add(AutoActions.MOVE, (int) spikePos[0], 0, 0.0);
//        beginning.add(MOVE, (spikePos));
        beginning.add(AutoActions.PLACE_PIXEL);
        beginning.add(AutoActions.MOVE,(int) backDropPos[0], (int) spikePos[1], 0.0);
//        beginning.add(MOVE, backDropPos));
        beginning.add(AutoActions.DELIVER);

        return beginning;
    }

    public static AutoActionHandler getBeginningNearPixels(Robot r, Telemetry t){
        beginning = new AutoActionHandler(r, t);

        double[] spikePos = new double[3];
        double[] backDropPos = new double[3];

        if (SPIKE_ZONE == 1){
            spikePos = LEFT_SPIKE_POS;
            backDropPos = LEFT_BACKDROP_POS;
        } else if (SPIKE_ZONE == 2) {
            spikePos = CENTER_SPIKE_POS;
            backDropPos = CENTER_BACKDROP_POS;
        }else{
            spikePos = RIGHT_SPIKE_POS;
            backDropPos = RIGHT_BACKDROP_POS;
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
        beginning.add(AutoActions.MOVE, backDropPos);
        beginning.add(AutoActions.DELIVER);

        return beginning;
    }

}
