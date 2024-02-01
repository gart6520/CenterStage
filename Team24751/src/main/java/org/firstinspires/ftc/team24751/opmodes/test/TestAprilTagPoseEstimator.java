package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorAprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Test AprilTag Pose Estimator", group = "Test")
public class TestAprilTagPoseEstimator extends LinearOpMode {
    Camera fieldCamera = new Camera(BACK_CAMERA_NAME, this);
    PoseEstimatorAprilTagProcessor aprilTag = new PoseEstimatorAprilTagProcessor(fieldCamera, this);
    Drivebase drive = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag.initAprilTagProcessor();
        fieldCamera.buildCamera();
        drive = new Drivebase(this);
        waitForStart();
        while (opModeIsActive()) {
            Pose2d odoPose = drive.getPoseEstimate();
            Vector2d aprilTagPos = aprilTag.getCurrentPosFromAprilTag(Math.toDegrees(odoPose.getHeading()));

            if (aprilTagPos == null) {
                telemetry.addLine("No April Tag");
            } else
                telemetry.addLine(String.format("\nAbs XY %6.1f, %6.1f", aprilTagPos.getX(), aprilTagPos.getY()));
            
            telemetryAprilTag();
            drive.manualControl(true);
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getCurrentDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XY %6.1f %6.1f (inch)", detection.ftcPose.x, detection.ftcPose.y));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


}