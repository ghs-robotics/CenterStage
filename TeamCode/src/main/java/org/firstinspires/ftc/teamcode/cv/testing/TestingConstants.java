package org.firstinspires.ftc.teamcode.cv.testing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TestingConstants {
    public static int RES_HEIGHT = 240;
    public static int RES_WIDTH = 320;

    // found HSV constants but need to find optimal difference
    public static int BLOCK_DARK_H = 70;
    public static int BLOCK_LIGHT_H = 0;

    public static int BLOCK_DARK_S = 255;
    public static int BLOCK_LIGHT_S = 100;

    public static int BLOCK_DARK_V = 200;
    public static int BLOCK_LIGHT_V = 40;

    public static int STRICT_LOWER_H = 0;
    public static int STRICT_LOWER_S = 60;
    public static int STRICT_LOWER_V = 0;

    public static int STRICT_UPPER_H = 190;
    public static int STRICT_UPPER_S = 230;
    public static int STRICT_UPPER_V = 100;

    public static boolean FILTER = true;
    public static boolean CANNY = true;

}
