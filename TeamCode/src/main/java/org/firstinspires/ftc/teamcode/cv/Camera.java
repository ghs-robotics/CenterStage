package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public TeamPropPipeline pipeline;

    public OpenCvCamera cam;

    private final int PIXEL_HEIGHT = 240;
    private final int PIXEL_WIDTH = 320;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new TeamPropPipeline(cam, telemetry);
    }

    public void initCamera(){
        cam.setPipeline(pipeline);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(PIXEL_WIDTH, PIXEL_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error: Camera could not open");
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    public void closeCamera(){
        cam.stopStreaming();
        cam.closeCameraDevice();
    }

    public void camTelemetry(){
        telemetry.addLine();
        telemetry.addData("Frame Count", cam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", cam.getFps()));
        telemetry.addData("Total frame time ms", cam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", cam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", cam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", cam.getCurrentPipelineMaxFps());
        telemetry.addLine();
    }

    public int getZone(){
        return pipeline.getZone();
    }

}