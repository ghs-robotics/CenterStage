package org.firstinspires.ftc.teamcode.cv.testing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TestingConstants {
    public static int PIXEL_HEIGHT = 240;
    public static int PIXEL_WIDTH = 320;

    // found HSV constants but need to find optimal difference
    public static int BLOCK_DARK_H = 165;
    public static int BLOCK_LIGHT_H = 100;

    public static int BLOCK_DARK_S = 130;
    public static int BLOCK_LIGHT_S = 109;

    public static int BLOCK_DARK_V = 255;
    public static int BLOCK_LIGHT_V = 200;

    public static boolean FILTER = true;
    public static boolean CANNY = false;

}
