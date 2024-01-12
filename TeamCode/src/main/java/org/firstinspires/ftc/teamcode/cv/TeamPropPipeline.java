package org.firstinspires.ftc.teamcode.cv;

import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_FIRST_LOWER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_FIRST_LOWER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_FIRST_UPPER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_FIRST_UPPER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_STRICT_LOWER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_STRICT_LOWER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_STRICT_UPPER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.H_STRICT_UPPER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_FIRST_LOWER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_FIRST_LOWER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_FIRST_UPPER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_FIRST_UPPER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_STRICT_LOWER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_STRICT_LOWER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_STRICT_UPPER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.S_STRICT_UPPER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_FIRST_LOWER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_FIRST_LOWER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_FIRST_UPPER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_FIRST_UPPER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_STRICT_LOWER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_STRICT_LOWER_RED;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_STRICT_UPPER_BLUE;
import static org.firstinspires.ftc.teamcode.cv.CVConstants.V_STRICT_UPPER_RED;
import static org.firstinspires.ftc.teamcode.cv.Camera.SPIKE_ZONE;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_DARK_H;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_DARK_S;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_DARK_V;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_LIGHT_H;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_LIGHT_S;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_LIGHT_V;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.CANNY;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.FILTER;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.RES_HEIGHT;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.RES_WIDTH;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.STRICT_LOWER_H;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.STRICT_LOWER_S;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.STRICT_LOWER_V;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.STRICT_UPPER_H;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.STRICT_UPPER_S;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.STRICT_UPPER_V;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
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

public class TeamPropPipeline extends OpenCvPipeline {
    boolean viewportPaused = false;

    OpenCvCamera cam;

    Telemetry telemetry;

    int spikeZone = -1;

    Scalar lightRange = new Scalar(BLOCK_LIGHT_H, BLOCK_LIGHT_S, BLOCK_LIGHT_V);
    Scalar darkRange = new Scalar(BLOCK_DARK_H, BLOCK_DARK_S, BLOCK_DARK_V);

    Scalar firstFilterLower;
    Scalar firstFilterUpper;

    Scalar strictLowerFilter;
    Scalar strictUpperFilter;

    ElapsedTime timer;

    int z1Pixels;
    int z2Pixels;
    int z3Pixels;

    // true == red, false == blue
    public TeamPropPipeline(OpenCvCamera camera, Telemetry telemetry, boolean color) {
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

        Core.bitwise_and(hsv, hsv, masked, thresh);

        Scalar avg = Core.mean(masked, thresh);

        masked.convertTo(scaledMask, -1, 150 / avg.val[1], 0);
        Core.inRange(scaledMask, strictLowerFilter, strictUpperFilter, scaledThresh);

        Core.bitwise_and(hsv, hsv, finalMat, scaledThresh);

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
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 2, true);
            boundingBox[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        double xLeft = RES_WIDTH / 3.5;
        double xRight = RES_WIDTH * 1.9 / 3;

        int zone = 0;
        int zone1Counter = 0;
        int zone2Counter = 0;
        int zone3Counter = 0;

        for (int i = 0; i != boundingBox.length; i++){
            if (boundingBox[i].x < xLeft)
                zone1Counter++;
            else if (boundingBox[i].x > xLeft && boundingBox[i].x + boundingBox[i].width < xRight)
                zone2Counter++;
            else
                zone3Counter++;

            Imgproc.rectangle(scaledThresh, boundingBox[i], new Scalar(0.5, 76.9, 89.8));
        }

        z1Pixels = zone1Counter;
        z2Pixels = zone2Counter;
        z3Pixels = zone3Counter;

        if (zone1Counter > zone2Counter && zone1Counter > zone3Counter)
            zone = 1;
        else if (zone2Counter > zone3Counter)
            zone = 2;
        else
            zone = 3;

//            if (timer.milliseconds() < 1000)
        SPIKE_ZONE = zone;

        Rect left = new Rect(1, 1, (int) xLeft, RES_HEIGHT);
        Rect center = new Rect((int) xLeft, 1, (int) (xRight - xLeft), RES_HEIGHT);
        Imgproc.rectangle(scaledThresh, left, new Scalar(255, 0, 0), 3);
        Imgproc.rectangle(scaledThresh, center, new Scalar(255, 0, 0), 3);

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
            firstFilterLower = new Scalar(H_FIRST_LOWER_RED, S_FIRST_LOWER_RED, V_FIRST_LOWER_RED);
            firstFilterUpper = new Scalar(H_FIRST_UPPER_RED, S_FIRST_UPPER_RED, V_FIRST_UPPER_RED);

            strictLowerFilter = new Scalar(H_STRICT_LOWER_RED, S_STRICT_LOWER_RED, V_STRICT_LOWER_RED);
            strictUpperFilter = new Scalar(H_STRICT_UPPER_RED, S_STRICT_UPPER_RED, V_STRICT_UPPER_RED);
        }else{
            firstFilterLower = new Scalar(H_FIRST_LOWER_BLUE, S_FIRST_LOWER_BLUE, V_FIRST_LOWER_BLUE);
            firstFilterUpper = new Scalar(H_FIRST_UPPER_BLUE, S_FIRST_UPPER_BLUE, V_FIRST_UPPER_BLUE);

            strictLowerFilter = new Scalar(H_STRICT_LOWER_BLUE, S_STRICT_LOWER_BLUE, V_STRICT_LOWER_BLUE);
            strictUpperFilter = new Scalar(H_STRICT_UPPER_BLUE, S_STRICT_UPPER_BLUE, V_STRICT_UPPER_BLUE);
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
