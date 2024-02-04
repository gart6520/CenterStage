package org.firstinspires.ftc.team24751.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.commands.AutoLockApriltagServo;
import org.firstinspires.ftc.team24751.subsystems.DroneLauncher;
import org.firstinspires.ftc.team24751.subsystems.Lift;
import org.firstinspires.ftc.team24751.subsystems.sensor.Distance;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.PoseEstimatorAprilTagProcessor;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.ROBOT_TO_CAMERA;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.BACK_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.CAMERA_SERVO;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.FRONT_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.GAMEPAD_SENSITIVITY.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.DroneLauncher.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Extender.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.*;
import static org.firstinspires.ftc.team24751.Utility.enableBulkRead;
import static org.firstinspires.ftc.team24751.Constants.FIELD_PARAMETER.*;

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
    Lift lift = new Lift(this);
    Camera frontCamera = new Camera(FRONT_CAMERA_NAME, this);
    Camera backCamera = new Camera(BACK_CAMERA_NAME, this);
    PoseEstimatorAprilTagProcessor aprilTag = new PoseEstimatorAprilTagProcessor(backCamera, this);
    AutoLockApriltagServo autoAimAprilTag = new AutoLockApriltagServo(CAMERA_SERVO, this);

    // States of the arm for FSM
    enum ArmState {
        none, base_moving, arm_moving_up, arm_moving_down, intaking, outaking, quick_reset
    }

    // Variable holding current arm FSM state
    ArmState state = ArmState.none;

    // Gamepad 1
    Gamepad prev1 = null;
    Gamepad curr1 = null;
    boolean grablt = true;
    boolean grabrt = true;

    // Gamepad 2
    Gamepad prev2 = null;
    Gamepad curr2 = null;

    // Timers for FSM
    ElapsedTime armMoveDownTimeout = new ElapsedTime();
    ElapsedTime armMoveUpTimeout = new ElapsedTime();
    ElapsedTime retractExtenderTimeout = new ElapsedTime();
    ElapsedTime droneLauncherHoldingTimer = new ElapsedTime();
    boolean isRetractExtenderTimeoutReset = false;

    // Helper function for dropping the arm down and reset
    private void dropArmAndReset() {
        ElapsedTime timer = new ElapsedTime();
        wrist.setAngle(GROUND_PARALLEL_DEG);
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
        wrist.setAngle(FULL_BACKWARD_DEG);
        arm.resetEncoder();
        arm.setPower(0);
        extender.resetPosition();

        // Set initial state to intaking
        // After drop arm and reset, the arm now should be at intake position
        state = ArmState.base_moving;
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
        lift.init();
        droneLauncher.init();
        autoAimAprilTag.init();
        aprilTag.initAprilTagProcessor();
        backCamera.buildCamera();

        // Enable bulk reads in auto mode
        enableBulkRead(hardwareMap);

        // Init gamepad objects
        prev1 = new Gamepad();
        prev1.copy(gamepad1);
        curr1 = new Gamepad();

        prev2 = new Gamepad();
        prev2.copy(gamepad2);
        curr2 = new Gamepad();

        // Load last pose from auto mode
        drivebase.setPoseEstimate(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        //Auto aim initially
        autoAimAprilTag.loop(getCameraPos(), Math.toDegrees(drivebase.getPoseEstimate().getHeading()));

        // Reset runtime
        runtime.reset();

        // Drop the arm down and reset arm angle
        dropArmAndReset();

        // Main loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update current gamepad state
            curr1.copy(gamepad1);
            curr2.copy(gamepad2);

            // Get pose from odometry pod
            Pose2d pose = drivebase.getPoseEstimate();

            //Auto Aim apriltag
            autoAimAprilTag.loop(getCameraPos(), Math.toDegrees(pose.getHeading()));

            // Control drivebase manually using joystick (field-oriented)
            this.manualDrive(aprilTag.getCurrentPosFromAprilTag(Math.toDegrees(pose.getHeading()) + 180));

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
                    wrist.setAngle(FULL_BACKWARD_DEG);

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
                        arm.setTargetAngle(ARM_PARALLEL_ANGLE);
                        arm.resetPID();

                        retractExtenderTimeout.reset();
                    }

                    break;
                case arm_moving_up:
                    // This state will move the arm up to outtake position

                    // If arm is extended -> retract it before moving
                    // Or the arm's PID will get crazy
                    if (extender.getPosition() > EXTENDER_FULLY_IN_THRESHOLD && armMoveUpTimeout.seconds() < 1) {
                        extender.setPower(0.7);
                        isRetractExtenderTimeoutReset = true;
                    }
                    // When done retracing the arm's extender -> stop extender motor
                    else if (armMoveUpTimeout.seconds() < 1.5 || extender.getPosition() <= EXTENDER_FULLY_IN_THRESHOLD) {
                        extender.setPower(0);
                        isRetractExtenderTimeoutReset = false;
                    }

                    // Move grabber up
                    wrist.setAngle(FULL_BACKWARD_DEG);

                    // Use arm PID to move arm to desired angle
                    if (arm.anglePIDLoop() || armMoveUpTimeout.seconds() > 1.75) {
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
                    wrist.setAngle(GROUND_PARALLEL_DEG);
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
                        armMoveDownTimeout.reset();
                    }

                    // If arm switch state button is pressed -> switch to arm_moving_up state
                    // (Technically moving arm up to outtake position)
                    else if (armButton) {
                        state = ArmState.arm_moving_up;
                        armMoveUpTimeout.reset();
                        arm.setTargetAngle(ARM_PARALLEL_ANGLE);
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

                    // If arm angle is < 90
                    if (angle < 90) {
                        // If just enter 90 degree state
                        if (!isRetractExtenderTimeoutReset) {
                            // Retract extender
                            retractExtenderTimeout.reset();
                            isRetractExtenderTimeoutReset = true;
                            extender.setPower(0.7);

                            // Move wrist up to allow base moving
                            wrist.setAngle(GROUND_PARALLEL_DEG);

                            // Hold the arm, waiting for extender and wrist to complete their job
                            //arm.setPower(0);
                        }
                        arm.setPower(-0.1);
                        if (angle <= 20) {
                            arm.setPower(-0.05);
                        }

                        // If extender is fully retracted
                        if (extender.getPosition() < EXTENDER_FULLY_IN_THRESHOLD || retractExtenderTimeout.seconds() > 1) {
                            extender.setPower(0);
                            //extender.resetPosition();
                        }
                    }

                    // If angle > 90 -> move arm down
                    else {
                        arm.setPower(-0.7);
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
                        wrist.setAngle(FULL_BACKWARD_DEG);

                        // Switch to base_moving state
                        state = ArmState.base_moving;
                    }

                    break;
                case quick_reset:
                    // This state move the arm up and down a little bit to force the grabber to fully touch the ground

                    // Set wrist angle to correct position
                    wrist.setAngle(GROUND_PARALLEL_DEG);

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
                grablt = !grablt;
                grabber.leftClaw.setPosition(grablt ? 0.25 : 0);
                grabrt = !grabrt;
                grabber.rightClaw.setPosition(grabrt ? 0.25 : 0);
            } else {// Control left claw
                if (curr1.right_bumper && !prev1.right_bumper) {
                    grablt = !grablt;
                    grabber.leftClaw.setPosition(grablt ? 0.25 : 0);
                }

                // Control right claw
                if (curr1.left_bumper && !prev1.left_bumper) {
                    grabrt = !grabrt;
                    grabber.rightClaw.setPosition(grabrt ? 0.25 : 0);
                }
            }

            // Reset yaw
            if (curr1.share && !prev1.share) {
                drivebase.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), 0));
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
                lift.setPower(1);
            } else if (gamepad1.cross && !gamepad1.options) {
                // Down
                lift.setPower(-1);
            } else {
                // Stop
                lift.setPower(0);
            }

            // Update prev1 gamepad
            prev1.copy(curr1);

            // Update prev2 gamepad
            prev2.copy(curr2);

            // Show telemetry
            telemetry.addData("Current Arm Position (L)", arm.leftArmEncoder.getPosition());
            telemetry.addData("Current Arm Position (R)", arm.rightArmEncoder.getPosition());
            telemetry.addData("Current Arm Angle (L + R)", arm.getAngle());
            telemetry.addData("Current Distance to Backdrop", distance.getDistanceCM());
            telemetry.addData("FSM State", state.toString());
            telemetry.addData("Extend position", extender.getPosition());
            telemetry.addData("Robot Yaw", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        droneLauncher.setPosition(LOAD_DRONE_LAUNCHER_POSITION);
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

    private void manualDrive(Vector2d aprilTagPos) {
        //In case of no april tag
        if (aprilTagPos == null) {
            drivebase.manualControl(true);
            return;
        }
        for (Rect rect : LOW_SPEED_COORDS) {
            if (rect.isInside(aprilTagPos)) {
                drivebase.manualControlLimitSpeed(true);
                return;
            }
        }

        drivebase.manualControl(true);
    }

    Vector2d getCameraPos() {
        Pose2d pose = drivebase.getPoseEstimate();
        return new Vector2d(pose.getX(), pose.getY()).plus(ROBOT_TO_CAMERA);
    }
}
