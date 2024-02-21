package org.firstinspires.ftc.team24751.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.commands.AutoAimApriltagServo;
import org.firstinspires.ftc.team24751.subsystems.ClimberHolder;
import org.firstinspires.ftc.team24751.subsystems.DroneLauncher;
import org.firstinspires.ftc.team24751.subsystems.FullPoseEstimator;
import org.firstinspires.ftc.team24751.subsystems.LedIndicator;
import org.firstinspires.ftc.team24751.subsystems.Climber;
import org.firstinspires.ftc.team24751.subsystems.sensor.Distance;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;


import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorAprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.ROBOT_TO_CAMERA;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.BACK_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.CAMERA_SERVO;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.FRONT_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.LED_GREEN_LEFT;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.LED_GREEN_RIGHT;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.LED_GREEN_WRIST;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.LED_RED_LEFT;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.LED_RED_RIGHT;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.LED_RED_WRIST;
import static org.firstinspires.ftc.team24751.Constants.GAMEPAD_SENSITIVITY.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.ClimberHolder.HOLD_CLIMBER_HOLDER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.ClimberHolder.RELEASE_CLIMBER_HOLDER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.DroneLauncher.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Extender.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.*;
import static org.firstinspires.ftc.team24751.Constants.VISION.BACK_CAMERA_RESOLUTION;
import static org.firstinspires.ftc.team24751.Utility.enableBulkRead;
import static org.firstinspires.ftc.team24751.Constants.FIELD_PARAMETER.*;

import static org.firstinspires.ftc.team24751.Utility.*;

import android.annotation.SuppressLint;

import java.util.List;

@TeleOp(name = "Semi-auto Main", group = "Manual")
public class SemiAutoMain extends LinearOpMode {
    // Run timer (mostly used for practicing)
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystems
    Drivebase drivebase = null;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Extender extender = new Extender(this);
    Distance distance = new Distance(this);
    DroneLauncher droneLauncher = new DroneLauncher(this);
    Climber climber = new Climber(this);
    ClimberHolder climberHolder = new ClimberHolder(this);
    LedIndicator leftLED = new LedIndicator(this, LED_RED_LEFT, LED_GREEN_LEFT);
    LedIndicator rightLED = new LedIndicator(this, LED_RED_RIGHT, LED_GREEN_RIGHT);
    LedIndicator wristLED = new LedIndicator(this, LED_RED_WRIST, LED_GREEN_WRIST);
    Camera frontCamera = new Camera(FRONT_CAMERA_NAME, this);
    Camera backCamera = new Camera(BACK_CAMERA_NAME, this);
    PoseEstimatorAprilTagProcessor aprilTag = new PoseEstimatorAprilTagProcessor(backCamera, this);
    FullPoseEstimator poseEstimator;
    AutoAimApriltagServo autoAimAprilTag = new AutoAimApriltagServo(CAMERA_SERVO, this);

    // States of the arm for FSM
    enum ArmState {
        none, base_moving, arm_moving_up, arm_moving_down, intaking, outaking, quick_reset
    }

    // Variable holding current arm FSM state
    ArmState state = ArmState.none;

    // Gamepad 1
    Gamepad prev1 = null;
    Gamepad curr1 = null;
    boolean grabLeftClose = false;
    boolean grabRightClose = false;

    // Gamepad 2
    Gamepad prev2 = null;
    Gamepad curr2 = null;

    // Timers
    ElapsedTime armMoveDownTimeout = new ElapsedTime();
    ElapsedTime armMoveUpTimeout = new ElapsedTime();
    ElapsedTime retractExtenderTimeout = new ElapsedTime();
    ElapsedTime droneLauncherHoldingTimer = new ElapsedTime();
    ElapsedTime climberHolderHoldingTimer = new ElapsedTime();
    boolean isRetractExtenderTimeoutReset = false;

    // Hubs objects
    List<LynxModule> allHubs = null;

    // Helper function for dropping the arm down and reset
    private void dropArmAndReset() {
        ElapsedTime timer = new ElapsedTime();

        wristLED.setGreen();
        wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);

        timer.reset();
        telemetry.addLine("Resetting arm and extender");
        telemetry.update();

        while (timer.seconds() <= 5 && distance.getDistanceCM() > DISTANCE_TO_GROUND_THRESHOLD) {
            if (timer.seconds() < 0.5) {
//                extender.setPower(0.9);
            } else {
//                extender.setPower(0);
            }

            if (timer.seconds() >= 1.5) {
                arm.setPower(-0.02);
            }
        }

        wristLED.turnOff();
        wrist.setAngle(WRIST_FULL_BACKWARD_DEG);

        arm.resetEncoder();
        arm.setPower(0);
        extender.resetPosition();

        // Set initial state to intaking
        // After drop arm and reset, the arm now should be at intake position
        state = ArmState.base_moving;
        grabber.setPosition(grabLeftClose ? CLOSE_CLAW_POSITION : OPEN_CLAW_POSITION, grabRightClose ? CLOSE_CLAW_POSITION : OPEN_CLAW_POSITION);

    }

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Init all subsystems
        initFieldParameters();
        drivebase = new Drivebase(this);
        arm.init();
        wrist.init();
        grabber.init();
        extender.init();
        distance.init();
        climber.init();
        climberHolder.init();
        droneLauncher.init();
        autoAimAprilTag.init();
        leftLED.init();
        rightLED.init();
        wristLED.init();
        aprilTag.initAprilTagProcessor();
        backCamera.buildCamera(BACK_CAMERA_RESOLUTION);
        poseEstimator = new FullPoseEstimator(
                this::getBotPosFromAprilTag,
//                (double deg) -> null,
//                aprilTag::getCurrentPosFromAprilTag,
                drivebase::getPoseFuse, PoseStorage.getPose());

        // Enable bulk reads in auto mode
        allHubs = enableBulkRead(hardwareMap);

        // Init gamepad objects
        prev1 = new Gamepad();
        prev1.copy(gamepad1);
        curr1 = new Gamepad();

        prev2 = new Gamepad();
        prev2.copy(gamepad2);
        curr2 = new Gamepad();

        // Load last pose from auto mode
        drivebase.setPoseEstimate(PoseStorage.getPose());

        // Set driver angle
        drivebase.setDriverAngle(PoseStorage.getPose().getHeading());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        //Auto aim initially
        autoAimAprilTag.loop(getCameraPos(), Math.toDegrees(drivebase.getPoseFuse().getHeading()));

        // Reset runtime
        runtime.reset();

        // Update Lynx LED
        lynxLEDUpdate();

        // Drop the arm down and reset arm angle
        dropArmAndReset();

        // Main loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update current gamepad state
            curr1.copy(gamepad1);
            curr2.copy(gamepad2);

            // Get pose estimate
            poseEstimator.cameraAngle = autoAimAprilTag.getCameraAngleRel();
            Pose2d botPose = poseEstimator.update(autoAimAprilTag.getCameraAngleRel());

            drivebase.setPoseFuse(botPose);
            //Auto Aim apriltag
            autoAimAprilTag.loop(getCameraPos(), Math.toDegrees(botPose.getHeading()));

            // Control drivebase manually using joystick (field-oriented)
            Pose2d odoPose = drivebase.getPoseEstimate();
            Vector2d botPos = new Vector2d(odoPose.getX(), odoPose.getY());
            this.manualDrive(botPos);

            // Update the arm position with kalman filter
            arm.update();

            // Check if these buttons are just newly pressed
            boolean grabberButton = curr2.cross && !prev2.cross;
            boolean armButton = curr2.triangle && !prev2.triangle;
            boolean quickResetButton = curr2.square && !prev2.square;

            // Arm finite state machine
            switch (state) {
                case none:
                    // Impossible state
                    break;
                case base_moving:
                    // This state will move the grabber up to avoid touching the ground

                    // Stop the arm
                    arm.setPower(0);

                    // Move grabber up -> no longer blocking drivebase's movement
                    wristLED.turnOff();
                    wrist.setAngle(WRIST_FULL_BACKWARD_DEG);

                    // If arm is extended -> retract it before moving
                    if (extender.getPosition() > EXTENDER_FULLY_IN_THRESHOLD && retractExtenderTimeout.seconds() < 1) {
                        extender.setPower(1);
                        isRetractExtenderTimeoutReset = true;
                    }

                    // When done retracing the arm's extender -> stop extender motor
                    else if (retractExtenderTimeout.seconds() < 1.5) {
                        isRetractExtenderTimeoutReset = false;

                        // Allow extender control
                        extenderControl();
                    }

                    // If grabber button is pressed -> change state from base_moving to intaking
                    // (Technically just move the grabber down to intake position)
                    if (grabberButton) {
                        state = ArmState.intaking;
                    }

                    // If arm switch state button is pressed -> switch to arm_moving_up state
                    // (Technically moving arm up to outtake position)
                    else if (armButton) {
                        state = ArmState.arm_moving_up;
                        armMoveUpTimeout.reset();
                        arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE);
                        arm.resetPID();

                        retractExtenderTimeout.reset();
                    }

                    break;
                case arm_moving_up:
                    // This state will move the arm up to outtake position

                    // If arm is extended -> retract it before moving
                    // Or the arm's PID will get crazy
                    if (extender.getPosition() > EXTENDER_FULLY_IN_THRESHOLD && armMoveUpTimeout.seconds() < 1) {
                        extender.setPower(1);
                        isRetractExtenderTimeoutReset = true;
                    }
                    // When done retracing the arm's extender -> stop extender motor
                    else if (armMoveUpTimeout.seconds() < 1.5) {
                        extender.setPower(0);
                        isRetractExtenderTimeoutReset = false;
                    }

                    // Move grabber up
                    wristLED.turnOff();
                    wrist.setAngle(WRIST_FULL_BACKWARD_DEG);

                    // Use arm PID to move arm to desired angle
                    if (arm.outakePIDLoop() || armMoveUpTimeout.seconds() > 1.75) {
                        arm.setPower(0);
                        state = ArmState.outaking;
                    }
                    if (arm.getAngle() > 90) {
                        extenderControl();
                    }

                    break;
                case intaking:
                    // This state will move grabber to intake position

                    // Set wrist angle to intake position (fully touch the ground)
                    wristLED.setGreen();
                    wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);

                    // Allow extender control
                    extenderControl();

                    // If grabber's reset button is pressed -> switch to quick_reset state
                    // Used to force the grabber to fully touch the ground
                    if (quickResetButton) {
                        armMoveDownTimeout.reset();
                        state = ArmState.quick_reset;
                    }

                    // If grabber button is pressed -> change state from intaking to base_moving
                    // (Technically just move the grabber up to avoid touching the ground)
                    else if (grabberButton) {
                        state = ArmState.base_moving;
                        retractExtenderTimeout.reset();
                    }

                    // If arm switch state button is pressed -> switch to arm_moving_up state
                    // (Technically moving arm up to outtake position)
                    else if (armButton) {
                        state = ArmState.arm_moving_up;
                        armMoveUpTimeout.reset();
                        arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE);
                        arm.resetPID();

                        retractExtenderTimeout.reset();
                    }

                    break;
                case outaking:
                    // This state move grabber to outtake position

                    // Set grabber to auto parallel with backdrop
                    wrist.autoParallel(arm.getAngle());
                    extenderControl();

                    // If arm switch state button is pressed -> switch to arm_moving_down state
                    // (Technically moving arm down to intake position)
                    if (armButton) {
                        armMoveDownTimeout.reset();
                        state = ArmState.arm_moving_down;
                    }

                    // Buttons for further tuning the arm's angle (just in case)
                    if (gamepad2.right_trigger > SENSE_TRIGGER) {
                        arm.setPower(Math.pow(gamepad2.right_trigger, 5));
                    } else if (gamepad2.right_bumper) {
                        arm.setPower(-0.4);
                    } else {
                        arm.setPower(0);
                    }

                    break;
                case arm_moving_down:
                    // This state move the arm down to intake position

                    // Get current arm angle
                    double angle = arm.getAngle();

                    // If arm angle is < 20
                    if (angle < 20) {
                        arm.setPower(-0.05);
                    }

                    // If arm angle is < 90
                    else if (angle < 90) {
                        // If just enter 90 degree state
                        if (!isRetractExtenderTimeoutReset) {
                            // Retract extender
                            retractExtenderTimeout.reset();
                            isRetractExtenderTimeoutReset = true;
                            extender.setPower(1);

                            // Move wrist down to detect if arm has touched the ground
                            wristLED.setGreen();
                            wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);

                            // Stop the arm a little bit
                            arm.setPower(0);
                        }

                        // Continue to move arm down
                        arm.setPower(-0.1);

                        // If extender is fully retracted
                        if (extender.getPosition() < EXTENDER_FULLY_IN_THRESHOLD || retractExtenderTimeout.seconds() > 1.5) {
                            extender.setPower(0);
                        }
                    }

                    // If angle > 90 -> move arm down
                    else {
                        arm.setPower(-0.55);
                    }

                    // If distance sensor reported touching ground or if arm is timeout
                    if (armMoveDownTimeout.seconds() > 1 && (armMoveDownTimeout.seconds() > 3 || distance.getDistanceCM() <= DISTANCE_TO_GROUND_THRESHOLD)) {
                        // Stop arm
                        arm.setPower(0);

                        // Reset arm encoder
                        arm.resetEncoder();

                        // Reset boolean
                        isRetractExtenderTimeoutReset = false;

                        // Move wrist up
                        wristLED.turnOff();
                        wrist.setAngle(WRIST_FULL_BACKWARD_DEG);

                        // Switch to base_moving state
                        state = ArmState.base_moving;
                    }

                    break;
                case quick_reset:
                    // This state move the arm up and down a little bit to force the grabber to fully touch the ground

                    // Set wrist angle to correct position
                    wristLED.setGreen();
                    wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);

                    // Move arm up for 0.5 seconds
                    if (armMoveDownTimeout.seconds() < 0.5) {
                        arm.setPower(0.5);
                    }

                    // Then move arm down
                    else if (armMoveDownTimeout.seconds() < 3 && distance.getDistanceCM() >= DISTANCE_TO_GROUND_THRESHOLD) {
                        arm.setPower(-0.1);
                    }

                    // Finally, after fully touching the ground
                    else {
                        // Stop arm
                        arm.setPower(0);

                        // Reset arm encoder
                        arm.resetEncoder();

                        // Switch to intaking state
                        state = ArmState.intaking;
                    }

                    break;
            }

            /**
             * General buttons mapping for gamepad1
             */

            if (curr1.left_trigger > SENSE_TRIGGER && prev1.left_trigger <= SENSE_TRIGGER) {
                grabLeftClose = !grabLeftClose;
                grabRightClose = !grabRightClose;
                grabber.setPosition(grabLeftClose ? CLOSE_CLAW_POSITION : OPEN_CLAW_POSITION, grabRightClose ? CLOSE_CLAW_POSITION : OPEN_CLAW_POSITION);
            } else {// Control left claw
                if (curr1.left_bumper && !prev1.left_bumper) {
                    grabLeftClose = !grabLeftClose;
                    grabber.leftClaw.setPosition(grabLeftClose ? CLOSE_CLAW_POSITION : OPEN_CLAW_POSITION);
                }

                // Control right claw
                if (curr1.right_bumper && !prev1.right_bumper) {
                    grabRightClose = !grabRightClose;
                    grabber.rightClaw.setPosition(grabRightClose ? CLOSE_CLAW_POSITION : OPEN_CLAW_POSITION);
                }
            }

            // Back LED indicator
            if (grabLeftClose) leftLED.setAmber();
            else leftLED.turnOff();
            if (grabRightClose) rightLED.setAmber();
            else rightLED.turnOff();

            // Reset yaw
            if (curr1.share && !prev1.share) {
                drivebase.setPoseEstimate(new Pose2d(botPose.getX(), botPose.getY(), 0));
                drivebase.setDriverAngle(0);
            }

            // Launch drone
            if (curr1.circle) {
                if (droneLauncherHoldingTimer.seconds() >= 0.69) {
                    droneLauncher.setPosition(SHOOT_DRONE_LAUNCHER_POSITION);
                }
            } else {
                droneLauncherHoldingTimer.reset();
            }

            // Hang
            if (gamepad1.triangle) {
                // Up
                //climber.setPower(1);
                if (climberHolderHoldingTimer.seconds() >= 0.69) {
                    climberHolder.setPosition(RELEASE_CLIMBER_HOLDER_POSITION);
                }
            } else if (!gamepad1.triangle) {
                climberHolderHoldingTimer.reset();
            }

            if (gamepad1.cross && !gamepad1.options) {
                // Down
                climber.setPower(-1);
            } else {
                // Stop
                climber.setPower(0);
            }

            // Update prev1 gamepad
            prev1.copy(curr1);

            // Update prev2 gamepad
            prev2.copy(curr2);

            // Show telemetry
            telemetry.addData("Current arm angle (L + R)", arm.getAngle());
            telemetry.addData("Current distance", distance.getDistanceCM());
            telemetry.addData("FSM state", state.toString());
            telemetry.addData("Extend position", extender.getPosition());
            telemetry.addData("Robot Pose", botPose.toString());
            Vector2d botPosFromAprilTag = getBotPosFromAprilTag(Math.toDegrees(botPose.getHeading()) + autoAimAprilTag.getCameraAngleRel());
            if (botPosFromAprilTag != null)
                telemetry.addData("Robot pose from AprilTag", botPosFromAprilTag.toString());
            telemetry.addData("Robot pose from odo", drivebase.getPoseFuse());
            telemetry.addData("Camera pos", getCameraPos().toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetryAprilTag();
            telemetry.update();
        }

        //droneLauncher.setPosition(LOAD_DRONE_LAUNCHER_POSITION);
    }

    private void extenderControl() {
        // Control extender
        if (gamepad2.left_bumper || gamepad2.dpad_down) {
            extender.setPower(0.7);
        } else if (gamepad2.left_trigger > SENSE_TRIGGER || gamepad2.dpad_up) {
            extender.setPower(-0.7);
        } else if (!isRetractExtenderTimeoutReset) {
            extender.setPower(0);
        }
    }

    private void manualDrive(Vector2d botPos) {
        /*for (Rect rect : LOW_SPEED_COORDS) {
            if (rect.isInside(botPos)) {
                drivebase.manualControlLimitSpeed(true);
                return;
            }
        }*/

        drivebase.manualControl(true);
    }

    Vector2d getCameraPos() {
        Pose2d pose = drivebase.getPoseEstimate();
        Vector2d rotatedRobotToCamera = rotateVector(ROBOT_TO_CAMERA, pose.getHeading());
        return new Vector2d(pose.getX(), pose.getY()).plus(rotatedRobotToCamera);
    }

    Vector2d getBotPosFromAprilTag(double cameraAngle) {
        Vector2d cameraPos = aprilTag.getCurrentPosFromAprilTag(cameraAngle);
        if (cameraPos == null) return null;
        Pose2d pose = drivebase.getPoseFuse();
        Vector2d rotatedRobotToCamera = rotateVector(ROBOT_TO_CAMERA, pose.getHeading());
        return cameraPos.minus(rotatedRobotToCamera);
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
    }

    private void lynxLEDUpdate() {
        // Command for the first hub
        LynxSetModuleLEDColorCommand cmd0 = new LynxSetModuleLEDColorCommand(allHubs.get(0),
                (byte) 255,
                (byte) 0,
                (byte) 255);

        // Command for the second hub
        LynxSetModuleLEDColorCommand cmd1 = new LynxSetModuleLEDColorCommand(allHubs.get(1),
                (byte) 255,
                (byte) 0,
                (byte) 255);

        // Send command
        try {
            cmd0.send();
            cmd1.send();
        } catch (InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }
    }
}
