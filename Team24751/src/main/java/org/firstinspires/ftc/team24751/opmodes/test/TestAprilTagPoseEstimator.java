package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorApriltagProcessor;

import java.util.List;

public class TestAprilTagPoseEstimator extends LinearOpMode {

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

        // Wait for the driver to press PLAY
        waitForStart();

        //Init gyro
        Gyro gyro = new Gyro(this);
        gyro.init();

        //Init camera
        Camera fieldCamera = new Camera(FIELD_CAMERA_NAME, this);

        //Init processor
        PoseEstimatorApriltagProcessor poseEstimator = new PoseEstimatorApriltagProcessor(fieldCamera, this);
        poseEstimator.initAprilTagProcessor();

        //Build camera
        //! Remember to init all processor before building camera
        fieldCamera.buildCamera();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            Vector2d pos = poseEstimator.getCurrentPoseFromApriltag(gyro.getYawDeg());
            telemetry.addData("Pose X-Y-Theta",
                    pos.x + "-" + pos.y + "-" + gyro.getYawDeg());
            telemetry.update();
        }
    }
}
