package org.firstinspires.ftc.teamcode.presets;

import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DELIVER;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.DETECT;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.MOVE_TO_SPIKE;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.PLACE_PIXEL;
import static org.firstinspires.ftc.teamcode.control.auto_execution.AutoActions.WAIT;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.BACK_ADJUST_Y;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.CENTER_PARKING;
import static org.firstinspires.ftc.teamcode.presets.AutoPositionPresets.CORNER_PARKING;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.control.auto_execution.AutoActionHandler;

public class AutoPresets {
    private static AutoActionHandler actionHandler;
    private static AutoActionHandler beginning;

//    static int pixelStackX = 3 * TICKS_PER_TILE_X + 20;
//    static int pixelStackY = (int) (1.2 * TICKS_PER_TILE_Y);


    public static AutoActionHandler goToSpikeMark(Robot r, Telemetry t){
        beginning = null;
        beginning = new AutoActionHandler(r, t);

        beginning.add(DETECT);
        beginning.add(MOVE, AutoPositionPresets.HALF_TO_MARK);
        beginning.add(MOVE_TO_SPIKE);
        beginning.add(PLACE_PIXEL);

        return beginning;
    }

    public static AutoActionHandler basicShort(Robot r, Telemetry t){
        actionHandler = null;
        actionHandler = new AutoActionHandler(r, t);

        actionHandler.add(goToSpikeMark(r, t));
        actionHandler.add(WAIT, 0.1);
        actionHandler.add(MOVE_TO_BACKDROP);
        actionHandler.add(DELIVER);
        actionHandler.add(WAIT, 0.1);
        return actionHandler;
    }

    public static AutoActionHandler basicLong(Robot r, Telemetry t){
        actionHandler = null;
        actionHandler = new AutoActionHandler(r, t);

        actionHandler.add(goToSpikeMark(r, t));
        actionHandler.add(WAIT, 0.1);
        actionHandler.add(MOVE_TO_BACKDROP);
        actionHandler.add(DELIVER);
        actionHandler.add(WAIT, 0.1);
        return actionHandler;
    }

    public static AutoActionHandler toBackDropAlongWall(Robot r, Telemetry t){
        actionHandler = null;
        actionHandler = new AutoActionHandler(r, t);

        actionHandler.add(MOVE, 15, 0, 0.0);
        return actionHandler;
    }

}
