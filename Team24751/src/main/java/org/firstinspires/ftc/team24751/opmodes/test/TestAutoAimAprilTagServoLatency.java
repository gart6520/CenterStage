package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.commands.AutoLockApriltagServo;
import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorApriltagProcessor;

import java.util.List;

@TeleOp(name = "Test Auto Aim April Tag Servo Latency", group = "test")
public class TestAutoAimAprilTagServoLatency extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        AutoLockApriltagServo autoServo = new AutoLockApriltagServo("servo", this);
        autoServo.initServo();
        //autoServo.getServo().getServo().setPosition(0);

        Camera cam = new Camera("fieldCamera", this);
        PoseEstimatorApriltagProcessor apriltagProcessor = new PoseEstimatorApriltagProcessor(cam, this);
        apriltagProcessor.initAprilTagProcessor();
        cam.buildCamera();

        Gyro gyro = new Gyro(this);
        gyro.init();

        // Init drivebase
        Drivebase drivebase = new Drivebase(this, gyro);
        drivebase.init();
        // Wait for the driver to press PLAY
        waitForStart();

        runtime.reset();
        // Init gyro (declaration above)
        gyro.reset();
        // Load last pose from auto mode
        drivebase.setCurrentPose(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double servoPos = 0;
        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Control drivebase manually
            double speed = gamepad1.right_trigger > 0.15 ? 1 : 0.5;

            // Get joystick axis values
            // Left joystick is used for driving bot in up/down/left/right direction, while right joystick is used for rotating the bot
            double left_y = -gamepad1.left_stick_y * DRIVEBASE_SPEED_Y * speed; // Y axis is inverted
            double left_x = gamepad1.left_stick_x * DRIVEBASE_SPEED_X * speed;
            double right_x = gamepad1.right_stick_x * DRIVEBASE_SPEED_Z * speed;

            // Drive
            // drivebase.drive(left_x, left_y, right_x); // Drive bot-oriented
            drivebase.driveFieldOriented(left_x, left_y, right_x); // Drive field-oriented

            //Pose estimation
            Vector2d camPos = apriltagProcessor.getCurrentPoseFromApriltag(getCamAngleDeg(autoServo, gyro));
            if (camPos != null) {
                telemetry.addData("Pose X-Y-Theta",
                        "\n" + camPos.x + "\n" + camPos.y + "\n" + getCamAngleDeg(autoServo, gyro));
            } else {
                telemetry.addData("Pose X-Y-Theta",
                        "No detection, " + getCamAngleDeg(autoServo, gyro));
            }
            telemetry.addData("Cam Angle", getCamAngleDeg(autoServo, gyro));
            telemetry.update();
            if (camPos == null || runtime.seconds() < 2)
                camPos = new Vector2d(0, 0);

            //Manual servo control
            if (gamepad1.dpad_right) {
                servoPos += 0.01;
                autoServo.getServo().getServo().setPosition(servoPos);
            } else if (gamepad1.dpad_left) {
                servoPos -= 0.01;
                autoServo.getServo().getServo().setPosition(servoPos);
            }
        }
    }

    private static double getCamAngleDeg(AutoLockApriltagServo autoServo, Gyro gyro) {
        return gyro.getYawDeg() + autoServo.getServo().getAngle();
    }
}
