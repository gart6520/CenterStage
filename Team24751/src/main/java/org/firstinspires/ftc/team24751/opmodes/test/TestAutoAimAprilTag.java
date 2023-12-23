package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.FIELD_PARAMETER.INIT_FIELD_PARAMETER;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;
import static org.firstinspires.ftc.team24751.Constants.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.commands.AutoLockApriltagServo;
import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorApriltagProcessor;

import java.util.List;
@TeleOp(name = "Test Auto Aim AprilTag", group = "Test")
public class TestAutoAimAprilTag extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoLockApriltagServo autoServo = new AutoLockApriltagServo("servo", this);
        autoServo.initServo();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        Camera cam = new Camera("fieldCamera", this);

        PoseEstimatorApriltagProcessor poseEstimator = new PoseEstimatorApriltagProcessor(cam, this);
        poseEstimator.initAprilTagProcessor();

        cam.buildCamera();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the driver to press PLAY
        waitForStart();

        // Init gyro
        Gyro gyro = new Gyro(this);
        gyro.init();

        // Init drivebase
        Drivebase drivebase = new Drivebase(this, gyro);
        drivebase.init();

        // Load last pose from auto mode
        drivebase.setCurrentPose(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Control drivebase manually
            // Get speed

            double speed = gamepad1.right_trigger > 0.15 ? 1 : 0.5;

            // Get joystick axis values
            // Left joystick is used for driving bot in up/down/left/right direction, while right joystick is used for rotating the bot
            double left_y = -gamepad1.left_stick_y * DRIVEBASE_SPEED_Y * speed; // Y axis is inverted
            double left_x = gamepad1.left_stick_x * DRIVEBASE_SPEED_X * speed;
            double right_x = gamepad1.right_stick_x * DRIVEBASE_SPEED_Z * speed;

            // Drive
            // drivebase.drive(left_x, left_y, right_x); // Drive bot-oriented
            drivebase.driveFieldOriented(left_x, left_y, right_x); // Drive field-oriented

            autoServo.loop(robotToCamera, gyro.getYawDeg());

            Vector2d pos = poseEstimator.getCurrentPoseFromApriltag(gyro.getYawDeg());
            if (pos != null) {
                telemetry.addData("Pose X-Y-Theta",
                        "\n" + pos.x + "\n" + pos.y + "\n" + gyro.getYawDeg());
            }

            else {
                telemetry.addData("Pose X-Y-Theta",
                        "No detection, " + gyro.getYawDeg());
            }

            telemetry.update();
        }
    }
}
