/*
 * Main manual drive OpMode for GART 24751's FTC code
 * Written by gvl610
 * Date created: 9/11/2023
 */

package org.firstinspires.ftc.team24751.opmodes.test;

// Import modules

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.FullPoseEstimator;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorAprilTagProcessor;

import java.util.List;

@TeleOp(name = "Test Full Pose Estimation", group = "Test")
public class TestFullPoseEstimation extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystems
    private Drivebase drivebase;
    private Camera fieldCamera = new Camera(BACK_CAMERA_NAME, this);
    private PoseEstimatorAprilTagProcessor aprilTag = new PoseEstimatorAprilTagProcessor(fieldCamera, this);
    private FullPoseEstimator poseEstimator;


    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Init drivebase
        drivebase = new Drivebase(this);
        aprilTag.initAprilTagProcessor();
        fieldCamera.buildCamera();
        poseEstimator = new FullPoseEstimator(aprilTag::getCurrentPosFromAprilTag, drivebase::getPoseEstimate);

        // Load last pose from auto mode
        drivebase.setPoseEstimate(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // Reset runtime
        runtime.reset();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {

            // Control drivebase manually
            drivebase.manualControl(true);

            // Show odoPose estimation
            Pose2d botPose = poseEstimator.update();

            telemetry.addData("Pose", botPose.toString());
            drivebase.setPoseEstimate(botPose);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
