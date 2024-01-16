package org.firstinspires.ftc.teamcode.control.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.cv.testing.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

public class Camera {

    private Telemetry telemetry;

    public Pipeline pipeline;

    public OpenCvSwitchableWebcam camera;

    private WebcamName camLeft;
    private WebcamName camRight;

    private final int PIXEL_HEIGHT = 480;
    private final int PIXEL_WIDTH = 640;

    public static int SPIKE_ZONE = 0;

    private boolean red;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry, boolean color){
        this.telemetry = telemetry;
        this.red = color;

        camRight = hardwareMap.get(WebcamName.class, "Webcam 1");
        camLeft = hardwareMap.get(WebcamName.class, "Webcam 2");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
                .createSwitchableWebcam(cameraMonitorViewId, camLeft, camRight);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(PIXEL_WIDTH, PIXEL_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                telemetry.setMsTransmissionInterval(100);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Error: Camera could not open");
                telemetry.update();
            }
        });

        pipeline = new Pipeline(camera, telemetry, color);
        camera.setPipeline(pipeline);

    }

    public void closeCamera(){
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    public void getTelemetry(){
        telemetry.addLine();
        pipeline.getTelemetry();
        telemetry.addData("camera ", camera.getActiveCamera());
        telemetry.addLine();
    }

    public int getSpikeZone(){
        if (SPIKE_ZONE == 0)
            SPIKE_ZONE = pipeline.getZone();
        return pipeline.getZone();
    }

    public void detectProp(){
        pipeline.resetDectectionTimer();
    }

    public void switchCamera(boolean changeCam){
        if (changeCam) {
            if (camera.getActiveCamera().equals(camLeft))
                camera.setActiveCamera(camRight);
            else
                camera.setActiveCamera(camLeft);
        }
    }

    public void setCamera(){
        if (red)
            camera.setActiveCamera(camLeft);
        else
            camera.setActiveCamera(camRight);
    }

}