package org.firstinspires.ftc.teamcode.control.presets;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DETECT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE_TO_SPIKE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.PLACE_PIXEL;
import static org.firstinspires.ftc.teamcode.control.cv.Camera.SPIKE_ZONE;
import static org.firstinspires.ftc.teamcode.control.presets.Position.CENTER_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.CENTER_SPIKE_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.HALF_TO_MARK;
import static org.firstinspires.ftc.teamcode.control.presets.Position.LEFT_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.LEFT_SPIKE_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.RIGHT_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.control.presets.Position.RIGHT_SPIKE_POS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActionHandler;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions;

public class AutoPresets {
    private static AutoActionHandler actionHandler;
    private static AutoActionHandler beginning;

//    static int pixelStackX = 3 * TICKS_PER_TILE_X + 20;
//    static int pixelStackY = (int) (1.2 * TICKS_PER_TILE_Y);


    public static AutoActionHandler goToSpikeMark(Robot r, Telemetry t){
        beginning = null;
        beginning = new AutoActionHandler(r, t);

        beginning.add(DETECT);
        beginning.add(MOVE, HALF_TO_MARK);
        beginning.add(MOVE_TO_SPIKE);
        beginning.add(PLACE_PIXEL);

        return beginning;
    }

    public static AutoActionHandler toBackDropAlongWall(Robot r, Telemetry t){
        actionHandler = null;
        actionHandler = new AutoActionHandler(r, t);

        actionHandler.add(MOVE, 15, 0, 0.0);
        return actionHandler;
    }
}
