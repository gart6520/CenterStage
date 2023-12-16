package org.firstinspires.ftc.team24751.subsystems.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.Pair;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.team24751.Constants.SENSITIVITY.*;
import static org.firstinspires.ftc.team24751.Constants.*;

import java.util.ArrayList;
import java.util.List;

public class PoseEstimatorApriltagProcessor {

    AprilTagProcessor aprilTag;
    Camera camera;
    LinearOpMode linearOpMode;

    public PoseEstimatorApriltagProcessor(Camera camera, LinearOpMode linearOpMode) {
        this.camera = camera;
        this.linearOpMode = linearOpMode;
    }

    //Null if no result or result too unreliable
    public Vector2d getCurrentPoseFromApriltag(double botAngleDeg) {
        //Get april tag detection
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()) return null;
        float decisionMarginSum = 0;
        ArrayList<Pair<Vector2d, Float>> robotPoseResult = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;
            decisionMarginSum += detection.decisionMargin;
            //Storing the pose from the detection and its decision margin
            robotPoseResult.add(new Pair<>(getCameraPoseFromApriltagDetection(detection, botAngleDeg), detection.decisionMargin));
        }
        if (decisionMarginSum < MARGIN_DECISION_THRESHOLD) return null;
        Vector2d currentPose = new Vector2d(0, 0);
        //Weighted average of all pose from apriltag
        for (Pair<Vector2d, Float> result : robotPoseResult) {
            currentPose = currentPose.plus(result.fst.times(result.snd / decisionMarginSum));
        }
        return currentPose;
    }

    private Vector2d getCameraPoseFromApriltagDetection(AprilTagDetection detection, double botAngleDeg) {
        VectorF _pos = detection.metadata.fieldPosition;

        //Global Position of apriltag
        Vector2d aprilTagPos = new Vector2d(_pos.get(0), _pos.get(1));

        //Convert unit if needed, TODO: Determine unit and convert beforehand
        float conversionFactor = detection.metadata.distanceUnit == DistanceUnit.METER ? (float) M_TO_INCH : 1;
        Vector2d cameraToApriltag = new Vector2d((float) detection.ftcPose.x * conversionFactor, (float) detection.ftcPose.y * conversionFactor);

        //XY swapped between april tag reference frame and FTC reference frame
        double radians = Math.toRadians(-90 +botAngleDeg);
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);


        //Rotate cameraToApriltag vector from the robot perspective (robot based) to
        //global perspective (field based)
        Vector2d cameraToAprilTagWorld = new Vector2d(
                cameraToApriltag.x * cos - cameraToApriltag.y * sin,
                cameraToApriltag.x * sin + cameraToApriltag.y * cos
        );
        linearOpMode.telemetry.addData("Diff rel Pos", detection.id + ":\n" + cameraToApriltag.x + '\n' + cameraToApriltag.y);
        linearOpMode.telemetry.addData("Diff global Pos", detection.id + ":\n" + cameraToAprilTagWorld.x + '\n' + cameraToAprilTagWorld.y);
        Vector2d cameraPos = aprilTagPos.minus(cameraToAprilTagWorld);
        linearOpMode.telemetry.addData("Abs Pos", detection.id + ":\n" + cameraPos.x + '\n' + cameraPos.y);
        return cameraPos;
    }

    public void initAprilTagProcessor() {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal inside wrapper class
        camera.addProcessorToQueue(aprilTag);
    }
}
