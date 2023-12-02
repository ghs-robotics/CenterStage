package org.firstinspires.ftc.teamcode.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {

    int zone;

    public TeamPropPipeline (OpenCvCamera camera, Telemetry telemetry){

    }

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    public int getZone(){
        return zone;
    }
}
