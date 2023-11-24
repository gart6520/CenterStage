package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.team24751.Constants.SENSITIVITY.*;
import static org.firstinspires.ftc.team24751.Constants.*;

import android.util.Size;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

public class PoseEstimatorCamera {
    String cameraName;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    LinearOpMode linearOpMode;

    public PoseEstimatorCamera(String cameraName, LinearOpMode linearOpMode) {
        this.cameraName = cameraName;
        this.linearOpMode = linearOpMode;
    }

    //Null if no result or result too unreliable
    public Vector2d cameraLoop(double botAngle) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()) return null;
        float decisionMarginSum = 0;
        ArrayList<Pair<Vector2d, Float>> robotPoseResult = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;
            decisionMarginSum += detection.decisionMargin;
            robotPoseResult.add(new Pair<>(getCameraPoseFromApriltagDetection(detection, botAngle), detection.decisionMargin));
        }
        if (decisionMarginSum < MARGIN_DECISION_THRESHOLD) return null;
        Vector2d currentPose = new Vector2d(0, 0);
        for (Pair<Vector2d, Float> result : robotPoseResult) {
            currentPose = currentPose.plus(result.fst.times(result.snd / decisionMarginSum));
        }
        return currentPose;
    }

    private Vector2d getCameraPoseFromApriltagDetection(AprilTagDetection detection, double botAngle) {
        VectorF _pos = detection.metadata.fieldPosition;
        Vector2d pos = new Vector2d(_pos.get(0), _pos.get(1));
        Vector2d aprilTagPose = new Vector2d(pos.x, pos.y);
        float conversionFactor = detection.metadata.distanceUnit == DistanceUnit.METER ? (float) M_TO_INCH : 1;
        Vector2d cameraToApriltag = new Vector2d((float) detection.ftcPose.x * conversionFactor, (float) detection.ftcPose.y * conversionFactor);
        double radians = Math.toRadians(90 - botAngle);
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        Vector2d cameraToAprilTagWorld = new Vector2d(
                cameraToApriltag.x * cos - cameraToApriltag.y * sin,
                cameraToApriltag.x * sin + cameraToApriltag.y * cos
        );
        return aprilTagPose.minus(cameraToAprilTagWorld);
    }

    public void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();
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
                .setCamera(linearOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    /**
     * Remember to setManualExposure(6, 250); after init
     *
     * @param exposureMS lower -> less motion blue but darker image
     * @param gain       make image brighter artificially
     */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!linearOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                linearOpMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!linearOpMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                linearOpMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            linearOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            linearOpMode.sleep(20);
        }
    }
}
