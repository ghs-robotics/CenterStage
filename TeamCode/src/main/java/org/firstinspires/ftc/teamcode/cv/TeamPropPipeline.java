package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TeamPropPipeline extends OpenCvPipeline {
    private ArrayList<Integer> colorDetections;
    final int hueThreshold = 20;

    //OpenCV HSV
    final int[] lime = {38, 255, 255};
    final int[] magenta = {156, 255, 255};
    final int[] cyan = {95, 255, 255};
    final int[] hues = {38, 156, 95};
    private ArrayList<Double> hueTargets;

    public TeamPropPipeline(){

    }

    public Mat processFrame(Mat input){
        return input;
    }

    public int processImage(Mat input) {
        //Crop mat
        int leftCrop = (input.width() / 2) - 250;
        int rightCrop = (input.width() / 2) + 250;

        //Create left mat
        Rect rectLeft = new Rect(
                0,
                0,
                leftCrop - 1,
                input.height()
        );
        Mat leftMat = new Mat(input, rectLeft);

        //Create middle mat
        Rect rectMiddle = new Rect(
                leftCrop,
                0,
                rightCrop - leftCrop,
                input.height()
        );
        Mat middleMat = new Mat(input, rectMiddle);

        //Create right mat
        Rect rectRight = new Rect(
                rightCrop + 1,
                0,
                input.width() - (rightCrop + 1),
                input.height()
        );
        Mat rightMat = new Mat(input, rectRight);



        //convert all mats from RGB to HSV
        Mat leftMatHSV = new Mat();
        Imgproc.cvtColor(leftMat, leftMatHSV, Imgproc.COLOR_RGB2HSV);

        Mat middleMatHSV = new Mat();
        Imgproc.cvtColor(middleMat, middleMatHSV, Imgproc.COLOR_RGB2HSV);

        Mat rightMatHSV = new Mat();
        Imgproc.cvtColor(rightMat, rightMatHSV, Imgproc.COLOR_RGB2HSV);



        //Get all mats hues in threshold
        int leftInThreshold = 0;
        int middleInThreshold = 0;
        int rightInThreshold = 0;

        int targetHue = 0;
        int hueThreshold = 15;

        int pixelX;
        int pixelY;

        //Left hues in Threshold
        int matWidth = leftMatHSV.width();
        int matHeight = leftMatHSV.height();

        for (pixelX = 0; pixelX < matWidth; pixelX++){
            for (pixelY = 0; pixelY < matHeight; pixelY++){
                int hue = (int) leftMatHSV.get(pixelX, pixelY)[0];
                if (Math.abs((targetHue - hue + 540) % 360 - 180) <= hueThreshold ) leftInThreshold++;
            }
        }

        //Middle hues in Threshold
        matWidth = middleMatHSV.width();
        matHeight = middleMatHSV.height();

        for (pixelX = 0; pixelX < matWidth; pixelX++){
            for (pixelY = 0; pixelY < matHeight; pixelY++){
                int hue = (int) middleMatHSV.get(pixelX, pixelY)[0];
                if (Math.abs((targetHue - hue + 540) % 360 - 180) <= hueThreshold ) middleInThreshold++;
            }
        }

        //Right hues in Threshold
        matWidth = rightMatHSV.width();
        matHeight = rightMatHSV.height();

        for (pixelX = 0; pixelX < matWidth; pixelX++){
            for (pixelY = 0; pixelY < matHeight; pixelY++){
                int hue = (int) rightMatHSV.get(pixelX, pixelY)[0];
                if (Math.abs((targetHue - hue + 540) % 360 - 180) <= hueThreshold ) rightInThreshold++;
            }
        }

        //0 = left, 1 = middle, 2 = right
        int spikeMark;
        //Check greatest hues in Threshold
        if (leftInThreshold > rightInThreshold){
            if (leftInThreshold > middleInThreshold) spikeMark = 0;
            else spikeMark = 1;
        } else if (rightInThreshold > middleInThreshold) spikeMark = 2;
        else spikeMark = 1;


        return spikeMark;



        /*
        //Find average color of pixels
        Mat avgColorColumn = new Mat();
        Mat avgColor = new Mat();

        //Find average of each column
        Core.reduce(leftMatHSV, avgColorColumn, 0, Core.REDUCE_AVG);
        //Find average
        Core.reduce(avgColorColumn, avgColor, 1, Core.REDUCE_AVG);

        //Split HSV to get H
        ArrayList<Mat> channels = new ArrayList<Mat>(3);
        Core.split(avgColor, channels);
        Mat hueChannel = channels.get(0);

        hueTargets = new ArrayList<Double>();
        hueTargets.add(38.0);  //[ 38 ]
        hueTargets.add(156.0); //[ 156 ]
        hueTargets.add(95.0); //[ 195 ]

        //Compute distance from each target color hue
        ArrayList<Double> hueDiff = new ArrayList<Double>();
//
        for(int i=0; i<hueTargets.size(); i++) {
            hueDiff.add(Math.abs(hueTargets.get(i) - hueChannel.get(0, 0)[0]));
        }

//        Core.absdiff(hueTargets, hueChannel, hueDiff);
//
//        //Find location of min hue difference
        int result = -1;
        int minIndex = 2;
        double min = hueDiff.get(2);
        for(int i=1; i>=0; i--) {
            if(hueDiff.get(i) < min) {
                minIndex = i;
                min = hueDiff.get(i);
            }
        }

//        telemetry.addLine("Sighted Hue: " + hueChannel.get(0, 0)[0]);
//        telemetry.addLine("Hue Difference: " + hueDiff.get(minIndex));
//        telemetry.addLine("Result is: " + result);
//        telemetry.addLine("MinIndex is: " + minIndex);
//        telemetry.addLine("" + hueDiff.get(0));
//        telemetry.addLine("" + hueDiff.get(1));
//        telemetry.addLine("" + hueDiff.get(2));
//
//        telemetry.update();

        //Add color detection hue diff is small enough and not already detected
        if(hueDiff.get(minIndex) < hueThreshold ) {
            result = minIndex;
            boolean included = false;
            for (int detection : colorDetections)
                if (result == detection) included = true;
            if(!included) colorDetections.add(new Integer(result));
            //colorDetections.remove(0);
        }

        //Draw rectangle around center 30x30 pixels to help line up camera
        Point upperLeft = new Point((int)input.cols()/2 - 10, (int)input.cols()/2 + 10);
        Point bottomRight = new Point((int)input.cols()/2 + 10, (int)input.cols()/2 - 10);
        rectangle(input, upperLeft, bottomRight, new Scalar(255, 25, 25), 2);
        */
    }
}
