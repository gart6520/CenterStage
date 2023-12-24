package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

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

@TeleOp(name = "Test Auto Aim April Tag", group = "test")
public class TestAutoAimAprilTag extends LinearOpMode {

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

        Camera cam = new Camera("fieldCamera", this);
        PoseEstimatorApriltagProcessor apriltagProcessor = new PoseEstimatorApriltagProcessor(cam, this);
        apriltagProcessor.initAprilTagProcessor();
        cam.buildCamera();

        Gyro gyro = new Gyro(this);

        // Init drivebase
        Drivebase drivebase = new Drivebase(this, gyro);
        drivebase.init();
        // Wait for the driver to press PLAY
        waitForStart();

        // Init gyro (declaration above)
        gyro.init();

        // Load last pose from auto mode
        drivebase.setCurrentPose(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
            Vector2d robotPos = apriltagProcessor.getCurrentPoseFromApriltag(gyro.getYawDeg() + autoServo.getServo().getAngle());
            if (robotPos != null) {
                telemetry.addData("Pose X-Y-Theta",
                        "\n" + robotPos.x + "\n" + robotPos.y + "\n" + gyro.getYawDeg());
            } else {
                telemetry.addData("Pose X-Y-Theta",
                        "No detection, " + gyro.getYawDeg());
            }
            telemetry.update();
            if (robotPos == null) robotPos = new Vector2d(0, 0);

            //Auto aim april tag
            autoServo.loop(robotPos.plus(BOT_PARAMETERS.robotToCamera), gyro.getYawDeg());

        }
    }
}
