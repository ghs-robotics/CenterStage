package org.firstinspires.ftc.teamcode.cv.testing;

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

import android.provider.ContactsContract;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.Camera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.ml.ANN_MLP;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Pipeline extends OpenCvPipeline {
    boolean viewportPaused = false;

    OpenCvCamera cam;

    Telemetry telemetry;

    int spikeZone = -1;

    Scalar lightRange = new Scalar(BLOCK_LIGHT_H, BLOCK_LIGHT_S, BLOCK_LIGHT_V);
    Scalar darkRange = new Scalar(BLOCK_DARK_H, BLOCK_DARK_S, BLOCK_DARK_V);

    ElapsedTime timer;

    public Pipeline(OpenCvCamera camera, Telemetry telemetry) {
        cam = camera;
        this.telemetry = telemetry;
        timer = new ElapsedTime();

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

        Core.inRange(hsv, lightRange, darkRange, thresh);

        if (FILTER || CANNY) {
            Core.bitwise_and(hsv, hsv, masked, thresh);

            Scalar avg = Core.mean(masked, thresh);

            masked.convertTo(scaledMask, -1, 150 / avg.val[1], 0);

            Scalar strictLowerHSV = new Scalar(STRICT_LOWER_H, STRICT_LOWER_S, STRICT_LOWER_V);
            Scalar strictHighHSV = new Scalar(STRICT_UPPER_H, STRICT_UPPER_S, STRICT_UPPER_V);

            Core.inRange(scaledMask, strictLowerHSV, strictHighHSV, scaledThresh);

            Core.bitwise_and(hsv, hsv, finalMat, scaledThresh);

        }

        if (CANNY) {
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

            double xLeft = RES_WIDTH / 3.0;
            double xRight = RES_WIDTH * 2.0 / 3;

            int zone = 0;
            for (int i = 0; i != boundingBox.length; i++){
                if (boundingBox[i].x < xLeft)
                    zone = 1;
                else if (boundingBox[i].x > xLeft && boundingBox[i].x + boundingBox[i].width < xRight)
                    zone = 2;
                else
                    zone = 3;

                Imgproc.rectangle(scaledThresh, boundingBox[i], new Scalar(0.5, 76.9, 89.8));
            }

            if (timer.milliseconds() < 150000)
                SPIKE_ZONE = zone;
        }

        input.release();
        if (CANNY)
            scaledThresh.copyTo(input);
        else if (FILTER)
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


    public void getTelemetry(){
        telemetry.addLine("Pipeline telemetry");
        telemetry.addData("zone ", SPIKE_ZONE);
        telemetry.addLine();
    }

}