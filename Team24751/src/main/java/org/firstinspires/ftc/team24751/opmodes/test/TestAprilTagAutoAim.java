package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.ROBOT_TO_CAMERA;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.BACK_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.CAMERA_SERVO;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.commands.AutoAimApriltagServo;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorAprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Test AprilTag Auto Aim", group = "Test")
@Disabled
public class TestAprilTagAutoAim extends LinearOpMode {
    Camera fieldCamera = new Camera(BACK_CAMERA_NAME, this);
    PoseEstimatorAprilTagProcessor aprilTag = new PoseEstimatorAprilTagProcessor(fieldCamera, this);
    Drivebase drive = null;
    AutoAimApriltagServo autoLock = new AutoAimApriltagServo(CAMERA_SERVO, this);

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag.initAprilTagProcessor();
        fieldCamera.buildCamera();
        drive = new Drivebase(this);
        autoLock.init();
//        autoLock.getAngleServo().setAngle(0);

        //TODO rotate vector
        waitForStart();
        while (opModeIsActive()) {
            Pose2d odoPose = drive.getPoseEstimate();
            Vector2d odoPos = new Vector2d(odoPose.getX(), odoPose.getY());
            Vector2d aprilTagPos = aprilTag.getCurrentPosFromAprilTag(Math.toDegrees(odoPose.getHeading()));
            autoLock.loop(odoPos.plus(ROBOT_TO_CAMERA), Math.toDegrees(odoPose.getHeading()));

            if (aprilTagPos == null) {
                telemetry.addLine("No April Tag");
            } else
                telemetry.addLine(String.format("\nAbs XY %6.1f, %6.1f", aprilTagPos.getX(), aprilTagPos.getY()));

            telemetry.addData("Pose", odoPose.toString());

//            telemetryAprilTag();
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