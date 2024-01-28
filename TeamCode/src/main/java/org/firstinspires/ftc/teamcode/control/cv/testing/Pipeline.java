package org.firstinspires.ftc.teamcode.control.cv.testing;

import static org.firstinspires.ftc.teamcode.control.cv.Camera.SPIKE_ZONE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.presets.CVConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Pipeline extends OpenCvPipeline {
    boolean viewportPaused = false;

    OpenCvCamera cam;

    Telemetry telemetry;

    int spikeZone = 0;

    Scalar lightRange = new Scalar(TestingConstants.BLOCK_LIGHT_H, TestingConstants.BLOCK_LIGHT_S, TestingConstants.BLOCK_LIGHT_V);
    Scalar darkRange = new Scalar(TestingConstants.BLOCK_DARK_H, TestingConstants.BLOCK_DARK_S, TestingConstants.BLOCK_DARK_V);

    Scalar firstFilterLower;
    Scalar firstFilterUpper;

    Scalar strictLowerFilter;
    Scalar strictUpperFilter;

    ElapsedTime timer;

    int z1Pixels;
    int z2Pixels;
    int z3Pixels;

    // true == red, false == blue
    public Pipeline(OpenCvCamera camera, Telemetry telemetry, boolean color) {
        cam = camera;
        this.telemetry = telemetry;
        timer = new ElapsedTime();

        setRanges(color);

        timer.reset();
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat hsv = new Mat();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);


        if (hsv.empty())
            return input;

        Mat thresh = new Mat();
        Mat masked = new Mat();
        Mat scaledMask = new Mat();
        Mat scaledThresh = new Mat();
        Mat finalMat = new Mat();
        Mat edges = new Mat();

//        Core.inRange(hsv, lightRange, darkRange, thresh);
        Core.inRange(hsv, firstFilterLower, firstFilterUpper, thresh);

        if (TestingConstants.FILTER || TestingConstants.CANNY) {
            Core.bitwise_and(hsv, hsv, masked, thresh);

            Scalar avg = Core.mean(masked, thresh);

            masked.convertTo(scaledMask, -1, 150 / avg.val[1], 0);

            Scalar strictLowerHSV = new Scalar(TestingConstants.STRICT_LOWER_H, TestingConstants.STRICT_LOWER_S, TestingConstants.STRICT_LOWER_V);
            Scalar strictHighHSV = new Scalar(TestingConstants.STRICT_UPPER_H, TestingConstants.STRICT_UPPER_S, TestingConstants.STRICT_UPPER_V);

//            Core.inRange(scaledMask, strictLowerHSV, strictHighHSV, scaledThresh);
            Core.inRange(scaledMask, strictLowerFilter, strictUpperFilter, scaledThresh);

            Core.bitwise_and(hsv, hsv, finalMat, scaledThresh);

        }

        if (TestingConstants.CANNY) {
            Imgproc.Canny(finalMat, edges, 100, 200);


            //contours, apply post processing to information
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            //find contours, input scaledThresh because it has hard edges
            Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


            Rect[] boundingBox = new Rect[contours.size()];
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];

            for (int i = 0; i < contours.size(); i++){
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i],
                        2, true);
                boundingBox[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }

            double xLeft = TestingConstants.RES_WIDTH / 3.5;
            double xRight = TestingConstants.RES_WIDTH * 2.1 / 3;

            int zone = 0;
            int zone1Counter = 0;
            int zone2Counter = 0;
            int zone3Counter = 0;


            for (int i = 0; i != boundingBox.length; i++){

                if (boundingBox[i].x < xLeft )
                    zone1Counter ++;
                else if (boundingBox[i].x > xLeft && boundingBox[i].x + boundingBox[i].width < xRight)
                    zone2Counter ++;
                else
                    zone3Counter ++;

                Imgproc.rectangle(scaledThresh, boundingBox[i], new Scalar(0.5, 76.9, 89.8));
            }

            z1Pixels = zone1Counter;
            z2Pixels = zone2Counter;
            z3Pixels = zone3Counter;

            if (zone1Counter > zone2Counter && zone1Counter > zone3Counter)
                zone = 0;
            else if (zone2Counter > zone3Counter)
                zone = 1;
            else
                zone = 2;

            if (timer.milliseconds() < 250)
                SPIKE_ZONE = zone;

            Rect left = new Rect(1, 1, (int) xLeft, TestingConstants.RES_HEIGHT);
            Rect center = new Rect((int) xLeft, 1, (int) (xRight - xLeft), TestingConstants.RES_HEIGHT);
            Imgproc.rectangle(scaledThresh, left, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(scaledThresh, center, new Scalar(255, 0, 0), 2);

            Core.bitwise_and(hsv, hsv, finalMat, scaledThresh);
        }

        input.release();
        if (TestingConstants.CANNY)
            scaledThresh.copyTo(input);
        else if (TestingConstants.FILTER)
            scaledThresh.copyTo(input);
        else
            thresh.copyTo(input);



        hsv.release();
        thresh.release();
        masked.release();
        scaledMask.release();
        finalMat.release();
        edges.release();


        return input;
    }

    public int getZone() {
        return spikeZone;
    }


    @Override
    public void onViewportTapped() {
        viewportPaused = !viewportPaused;

        if(viewportPaused)
            cam.pauseViewport();
        else
            cam.resumeViewport();

    }

    // true is red, false is blue
    private void setRanges(boolean red){
        if (red){
            firstFilterLower = new Scalar(CVConstants.H_FIRST_LOWER_RED, CVConstants.S_FIRST_LOWER_RED, CVConstants.V_FIRST_LOWER_RED);
            firstFilterUpper = new Scalar(CVConstants.H_FIRST_UPPER_RED, CVConstants.S_FIRST_UPPER_RED, CVConstants.V_FIRST_UPPER_RED);

            strictLowerFilter = new Scalar(CVConstants.H_STRICT_LOWER_RED, CVConstants.S_STRICT_LOWER_RED, CVConstants.V_STRICT_LOWER_RED);
            strictUpperFilter = new Scalar(CVConstants.H_STRICT_UPPER_RED, CVConstants.S_STRICT_UPPER_RED, CVConstants.V_STRICT_UPPER_RED);
        }else{
            firstFilterLower = new Scalar(CVConstants.H_FIRST_LOWER_BLUE, CVConstants.S_FIRST_LOWER_BLUE, CVConstants.V_FIRST_LOWER_BLUE);
            firstFilterUpper = new Scalar(CVConstants.H_FIRST_UPPER_BLUE, CVConstants.S_FIRST_UPPER_BLUE, CVConstants.V_FIRST_UPPER_BLUE);

            strictLowerFilter = new Scalar(CVConstants.H_STRICT_LOWER_BLUE, CVConstants.S_STRICT_LOWER_BLUE, CVConstants.V_STRICT_LOWER_BLUE);
            strictUpperFilter = new Scalar(CVConstants.H_STRICT_UPPER_BLUE, CVConstants.S_STRICT_UPPER_BLUE, CVConstants.V_STRICT_UPPER_BLUE);
        }
    }

    public void resetDectectionTimer(){
        timer.reset();
    }

    public void getTelemetry(){
        telemetry.addLine("Pipeline telemetry");
        telemetry.addData("zone ", SPIKE_ZONE);
        telemetry.addData("zone 1 ", z1Pixels);
        telemetry.addData("zone 2 ", z2Pixels);
        telemetry.addData("zone 3 ", z3Pixels);
        telemetry.addLine();
    }

}