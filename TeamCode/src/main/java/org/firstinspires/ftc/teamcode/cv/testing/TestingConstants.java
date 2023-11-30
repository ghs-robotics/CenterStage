package org.firstinspires.ftc.teamcode.cv.testing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TestingConstants {
    public static int PIXEL_HEIGHT = 240;
    public static int PIXEL_WIDTH = 320;

    // found HSV constants but need to find optimal difference
    public static int BLOCK_DARK_H = 255;
    public static int BLOCK_LIGHT_H = 0;

    public static int BLOCK_DARK_S = 180;
    public static int BLOCK_LIGHT_S = 155;

    public static int BLOCK_DARK_V = 180;
    public static int BLOCK_LIGHT_V = 40;

    public static boolean FILTER = true;
    public static boolean CANNY = false;

}
