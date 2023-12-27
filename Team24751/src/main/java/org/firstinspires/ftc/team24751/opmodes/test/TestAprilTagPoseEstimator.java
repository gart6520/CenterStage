package org.firstinspires.ftc.team24751.opmodes.test;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.INIT_VALUE.INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.subsystems.AngleServo;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorApriltagProcessor;
import org.firstinspires.ftc.team24751.subsystems.vision.TeamPropProcessor;

import java.util.List;

@TeleOp(name = "Test AprilTag Pose Estimator", group = "Test")
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

        //Init camera
        Camera fieldCamera = new Camera(FIELD_CAMERA_NAME, this);

        AngleServo angleServo = new AngleServo("servo", INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG, 270, this);
        angleServo.init();
        angleServo.getServo().setDirection(Servo.Direction.REVERSE);

        //Init processor
        PoseEstimatorApriltagProcessor poseEstimator = new PoseEstimatorApriltagProcessor(fieldCamera, this);
        poseEstimator.initAprilTagProcessor();

//        TeamPropProcessor teamProp = new TeamPropProcessor();
//        fieldCamera.addProcessorToQueue(teamProp);

        //Build camera
        //! Remember to init all processor before building camera
        fieldCamera.buildCamera();

        // Wait for the driver to press PLAY
        waitForStart();

        //Init gyro
        Gyro gyro = new Gyro(this);
        gyro.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Loop, run until driver presses STOP
        double servoPos = 0;
        while (opModeIsActive()) {
            //Manual servo control
            if (gamepad1.dpad_right) {
                servoPos += 0.01;
                angleServo.getServo().setPosition(servoPos);
            } else if (gamepad1.dpad_left) {
                servoPos -= 0.01;
                angleServo.getServo().setPosition(servoPos);
            }
            servoPos = clamp(servoPos, 0 , 1);
            Vector2d pos = poseEstimator.getCurrentPoseFromApriltag(gyro.getYawDeg() + angleServo.getAngle());
            telemetry.addData("Servo angle", angleServo.getAngle());
            telemetry.addData("Servo PWM", angleServo.getServo().getPosition());
            if (pos != null) {
                telemetry.addData("Pose X-Y-Theta",
                        "\n" + pos.x + "\n" + pos.y + "\n" + (gyro.getYawDeg() + angleServo.getAngle()));
            } else {
                telemetry.addData("Pose X-Y-Theta",
                        "No detection, " + (gyro.getYawDeg() + angleServo.getAngle()));
            }
            telemetry.update();
//            Vector2d pos = teamProp.getCenterOfDetection();
//            telemetry.addData("Center of pixel", "\n" + pos.x + "\n" + pos.y);
        }
    }
}
