package org.firstinspires.ftc.teamcode.cv.testing;

import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_DARK_H;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_DARK_S;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_DARK_V;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_LIGHT_H;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_LIGHT_S;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.BLOCK_LIGHT_V;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.CANNY;
import static org.firstinspires.ftc.teamcode.cv.testing.TestingConstants.FILTER;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    boolean viewportPaused = false;

    OpenCvCamera cam;

    Telemetry telemetry;

    Mat hsv = new Mat();
    Mat display = new Mat();

    int spikeZone;

    public Pipeline (OpenCvCamera camera, Telemetry telemetry){
        cam = camera;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV, 3);

        display = processHSV(hsv);

        if (CANNY)
            Imgproc.Canny(display, display, 100, 122, 3, false);

        return display;
    }

    public int getZone(){
        return spikeZone;
    }

    private Mat processHSV(Mat input){
        Scalar lightRange = new Scalar(BLOCK_LIGHT_H, BLOCK_LIGHT_S, BLOCK_LIGHT_V);
        Scalar darkRange = new Scalar(BLOCK_DARK_S, BLOCK_DARK_H, BLOCK_DARK_V);

        if (FILTER)
            Core.inRange(input, lightRange, darkRange, input);

        return input;
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
        telemetry.addLine();
        telemetry.addLine("Pipeline telemetry");
        telemetry.addData("channels: ", display.channels());
        telemetry.addData("dump:     ", display.dump());
        telemetry.addLine();
    }

}