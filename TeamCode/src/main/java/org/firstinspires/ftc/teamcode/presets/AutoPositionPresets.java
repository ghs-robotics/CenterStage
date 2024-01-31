package org.firstinspires.ftc.teamcode.presets;

public class AutoPositionPresets {
    //
    private static final double SIDE_SPIKE_X = 665;
    private static final double IN_FRONT_OF_BACKDROP = 820;

    static final double X_TO_MM = 1.045553699;
    static final double Y_TO_MM = 1.040573807;

    public static final int TILE = 584;
    public static final int TILE_CENTER_X = 40;
    public static final int BACK_ADJUST_Y = 2 * TILE;

    // BACKSTAGE PARKING ------------------------------------------------------------------------------------------------------------

    public static double[] CORNER_PARKING = {100, 775, 0.0};
    public static double[] CENTER_PARKING = {1000, 775, 0};

    // BACK DROP + SPIKE MARK POSITIONS ---------------------------------------------------------------------------------------------

    public static double[] RED_LEFT_BACKDROP_POS = {730, IN_FRONT_OF_BACKDROP, 0.0};
    public static double[] RED_CENTER_BACKDROP_POS = {SIDE_SPIKE_X, IN_FRONT_OF_BACKDROP, 0.0};
    public static double[] RED_RIGHT_BACKDROP_POS = {480, IN_FRONT_OF_BACKDROP, 0.0};

    public static double[] BLUE_LEFT_BACKDROP_POS = RED_RIGHT_BACKDROP_POS;
    public static double[] BLUE_CENTER_BACKDROP_POS = {SIDE_SPIKE_X, IN_FRONT_OF_BACKDROP, 0.0};
    public static double[] BLUE_RIGHT_BACKDROP_POS = BLUE_LEFT_BACKDROP_POS;

    public static double[][] RED_BACKDROP_POS = {RED_LEFT_BACKDROP_POS, RED_CENTER_BACKDROP_POS, RED_RIGHT_BACKDROP_POS};
    public static double[][] BLUE_BACKDROP_POS = {BLUE_LEFT_BACKDROP_POS, BLUE_CENTER_BACKDROP_POS, BLUE_RIGHT_BACKDROP_POS};

    public static double[] BLUE_LEFT_SPIKE_POS = {SIDE_SPIKE_X, 480, 0.0};
    public static double[] BLUE_CENTER_SPIKE_POS = {850, 80, 0.0};
    public static double[] BLUE_RIGHT_SPIKE_POS = {SIDE_SPIKE_X, -35, 0.0};

    public static double[] RED_LEFT_SPIKE_POS = {SIDE_SPIKE_X, -35, 0.0};
    public static double[] RED_CENTER_SPIKE_POS = {850, 80, 0.0};
    public static double[] RED_RIGHT_SPIKE_POS = {SIDE_SPIKE_X, 480, 0.0};

    public static double[][] BLUE_SPIKE = {BLUE_LEFT_SPIKE_POS, BLUE_CENTER_SPIKE_POS, BLUE_RIGHT_SPIKE_POS};
    public static double[][] RED_SPIKE = {RED_LEFT_SPIKE_POS, RED_CENTER_SPIKE_POS, RED_RIGHT_SPIKE_POS};

    public static final double[] HALF_TO_MARK = {SIDE_SPIKE_X, 0, 0.0};

}
