package org.firstinspires.ftc.team24751.subsystems.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Camera {
    private VisionPortal camera;
    private final String cameraName;
    private final LinearOpMode linearOpMode;
    private final ArrayList<VisionProcessor> inQueueProcessors = new ArrayList<>();

    Camera(String cameraName, LinearOpMode linearOpMode) {
        this.cameraName = cameraName;
        this.linearOpMode = linearOpMode;
    }
    public void disableProcessor(VisionProcessor processor)
    {
        camera.setProcessorEnabled(processor, false);
    }
    /**
     * Add a processor to the list of processor that will be added to this camera when
     * .buildCamera() happen
     */
    public void addProcessorToQueue(VisionProcessor processor) {
        inQueueProcessors.add(processor);
    }

    /**
     * Remember to add all processor (aka init all camera related class) before build the camera
     */
    public void buildCamera() {
        VisionPortal.Builder cameraBuilder = new VisionPortal.Builder()
                .setCamera(linearOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        for (VisionProcessor processor : inQueueProcessors) {
            cameraBuilder.addProcessor(processor);
        }
        camera = cameraBuilder.build();
    }

    public VisionPortal getCamera() {
        return camera;
    }

    /**
     * Remember to setManualExposure(6, 250); after the camera has been built
     *
     * @param exposureMS lower -> less motion blue but darker image
     * @param gain       make image brighter artificially
     */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (camera == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (camera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!linearOpMode.isStopRequested() && (camera.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                linearOpMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!linearOpMode.isStopRequested()) {
            ExposureControl exposureControl = camera.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                linearOpMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            linearOpMode.sleep(20);
            GainControl gainControl = camera.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            linearOpMode.sleep(20);
        }
    }
}