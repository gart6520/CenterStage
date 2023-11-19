package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class PoseEstimatorCamera {
    String cameraName = "FieldWideCamera";
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;


    public VectorF cameraLoop() {
        float decisionMarginSum = 0;
        ArrayList<Pair<VectorF, Float>> robotPoseResult = new ArrayList<>();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;
            decisionMarginSum += detection.decisionMargin;
            robotPoseResult.add(new Pair<>(getCameraPoseFromApriltagDetection(detection), detection.decisionMargin));
        }
        VectorF currentPose = new VectorF(0, 0);
        for (Pair<VectorF, Float> result : robotPoseResult) {
            currentPose.add(result.fst.multiplied(result.snd / decisionMarginSum));
        }
        return currentPose;
    }

    private VectorF getCameraPoseFromApriltagDetection(AprilTagDetection detection) {
        VectorF pos = detection.metadata.fieldPosition;
        VectorF aprilTagPose = new VectorF(pos.get(0), pos.get(1));
        VectorF cameraToApriltag = new VectorF((float) detection.ftcPose.x, (float) detection.ftcPose.y);
        return aprilTagPose.subtracted(cameraToApriltag);
    }

    public void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .addProcessor(aprilTag)
                .build();
    }
//      setManualExposure(6, 250);
//    private void setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested()) {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//    }
}
