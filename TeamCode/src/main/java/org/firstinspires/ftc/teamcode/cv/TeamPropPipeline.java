package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TeamPropPipeline extends OpenCvPipeline {
    public TeamPropPipeline(){

    }

    @Override
    public Mat processFrame(Mat input) {
        //Crop image
        //Rect rectCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), (int) (input.width() / 5), (int) (input.height() / 5));
        //input = new Mat(input, rectCrop);

        //Sample Center Pixels
        //Rect sampleCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), 15, 15);
        //Mat colorSample = new Mat(input, sampleCrop);

        //convert image from RGB to HSV
        /*Mat hsvConvert = new Mat();
        cvtColor(colorSample, hsvConvert, Imgproc.COLOR_RGB2HSV);

        //Find average of each channel across sample pixels
        Mat avgColColor = new Mat();
        Mat avgColor = new Mat();
        Core.reduce(hsvConvert, avgColColor, 0, Core.REDUCE_AVG);
        Core.reduce(avgColColor, avgColor, 1, Core.REDUCE_AVG);
        ArrayList<Mat> channels = new ArrayList<Mat>(3);
        Core.split(avgColor, channels);
        Mat hueChannel = channels.get(0);

        hueTargets = new ArrayList<Double>();
        hueTargets.add(38.0);  //[ 38  ]
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

        //Draw rectangle around center 30x30 pixels to help line up cameraq

        Point upperLeft = new Point((int)input.cols()/2 - 10, (int)input.cols()/2 + 10);
        Point bottomRight = new Point((int)input.cols()/2 + 10, (int)input.cols()/2 - 10);
        rectangle(input, upperLeft, bottomRight, new Scalar(255, 25, 25), 2);

        return input;*/
        return null;
    }
}
