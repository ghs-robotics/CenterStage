package org.firstinspires.ftc.teamcode.cv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.testing.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {

    private Telemetry telemetry;

    public Pipeline pipeline;

    public OpenCvCamera camera1;
    public OpenCvCamera camera2;

    private final int PIXEL_HEIGHT = 240;
    private final int PIXEL_WIDTH = 320;


    public Camera(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        camera1 = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        camera2 = OpenCvCameraFactory.getInstance()
//                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));


        pipeline = new Pipeline(camera1, telemetry);
        camera1.setPipeline(pipeline);
    }

    public void initCamera(){
//        camera2.setPipeline(pipeline);

        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera1.startStreaming(PIXEL_WIDTH, PIXEL_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error: Camera could not open");
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(5);
    }

    public void closeCamera(){
        camera1.stopStreaming();
//        camera2.stopStreaming();
        camera1.closeCameraDevice();
//        camera2.closeCameraDevice();
    }

    public void getTelemetry(){
        telemetry.addLine();
        pipeline.getTelemetry();
        telemetry.addLine();
    }

    public int getSpikeZone(){
        return pipeline.getZone();
    }

}