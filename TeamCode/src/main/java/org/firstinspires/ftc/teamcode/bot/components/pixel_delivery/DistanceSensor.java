package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private final com.qualcomm.robotcore.hardware.DistanceSensor distance;

    private int pixels;

    private boolean pixelPresent;
    private final double[] pixelDistances = new double[8];
    public DistanceSensor (HardwareMap hardwareMap) {
        distance = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class,
                "distance");

        pixels = 0;
    }

    //-------------------------------------------------------------------------------------
    //                               Distance Sensor Functions
    //-------------------------------------------------------------------------------------

    /*
    when distance is < 7, pixels + 1
    read only once -> set entire array to 0
    when distance is >= 7, array can resume
    */

//    public void addInitialDataToDistanceArray() {
//        for (int i = 0; i < pixelDistances.length; i++) {
//            pixelDistances[i] = getDistanceFromPixel();
//        }
//    }
//
//    public void updatePixelDistances () {
//        for (int i = pixelDistances.length - 1; i > 0; i--) {
//            pixelDistances[i] = pixelDistances[i - 1];
//        }
//        pixelDistances[0] = getDistanceFromPixel();
//    }

    public int countPixels () {
        if (getDistanceFromPixel() < 7) {
            pixels++;
        }
        return pixels;
    }

//    public int countPixels() {
//        if (pixelDistances[0] > pixelDistances[pixelDistances.length - 1] + 1) {
//            pixels++;
//            clearHalfData();
//        }
//        return pixels;
//    }

//    public void countPixels (Intake intake) {
//        updatePixelDistances();
//        double current = pixelDistances[0];
//        double midpoint = pixelDistances[pixelDistances.length / 2];
//        double earliest = pixelDistances[pixelDistances.length - 1];
//
//        if (current == 0 || midpoint == 0 || earliest == 0 || current > 10.5
//                || intake.getIntakePower() == 0) {
//            return;
//        }
//        if ((midpoint > current) && (midpoint > earliest)) {
//            pixels += (intake.getIntakePower());
//        }
//    }
//
//    private void clearMiddleData() {
//        for (int i = (pixelDistances.length / 2) - 1; i < (pixelDistances.length / 2) + 1; i++) {
//            pixelDistances[i] = 5;
//        }
//    }

    public String pixelDistancesToString () {
        String dist = "";
        for (int i = 0; i < pixelDistances.length - 1; i++) {
            dist += pixelDistances[i] + ", ";
        }
        dist = dist + pixelDistances[pixelDistances.length - 1];

        return dist;
    }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    public int getPixelNumber () {
        return pixels;
    }

    public double getDistanceFromPixel () {
        return distance.getDistance(DistanceUnit.CM);
    }

    public String getPixelDistances () {
        return pixelDistancesToString();
    }
}
