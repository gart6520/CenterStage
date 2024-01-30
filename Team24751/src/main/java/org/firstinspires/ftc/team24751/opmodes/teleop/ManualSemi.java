package org.firstinspires.ftc.team24751.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.DroneLauncher;
import org.firstinspires.ftc.team24751.subsystems.Lift;
import org.firstinspires.ftc.team24751.subsystems.sensor.Distance;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

import static org.firstinspires.ftc.team24751.Constants.GAMEPAD_SENSITIVITY.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.DroneLauncher.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Extender.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.*;
import static org.firstinspires.ftc.team24751.Utility.enableBulkRead;

@TeleOp(name = "Semi-auto manual", group = "Manual")
public class ManualSemi extends LinearOpMode {
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

    // States of the arm for FSM
    enum ArmState {
        none, base_moving, arm_moving_up, arm_moving_down, intaking, outaking, quick_reset
    }

    // Variable holding current arm FSM state
    ArmState state = ArmState.none;

    // Gamepad 1
    Gamepad prev1 = null;
    Gamepad curr1 = null;
    boolean grablt = false;
    boolean grabrt = false;
    boolean droneLauncherShot = false;

    // Gamepad 2
    Gamepad prev2 = null;
    Gamepad curr2 = null;

    // Timers for FSM
    ElapsedTime armMoveDownTimeout = new ElapsedTime();
    ElapsedTime armMoveUpTimeout = new ElapsedTime();
    ElapsedTime retractExtenderTimeout = new ElapsedTime();
    boolean isRetractExtenderTimeoutReset = false;

    // Helper function for dropping the arm down and reset
    private void dropArmAndReset() {
        ElapsedTime timer = new ElapsedTime();
        wrist.setAngle(GROUND_PARALLEL_DEG);
        timer.reset();
        telemetry.addData("Timer", timer.seconds());
        telemetry.update();
        while (timer.seconds() <= 5 && distance.getDistanceCM() > DISTANCE_TO_GROUND_THRESHOLD) {
            if (timer.seconds() >= 1.5)
                arm.setPower(-0.4);
        }
        arm.resetEncoder();
        arm.setPower(0);
    }

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Init all subsystems
        drivebase = new Drivebase(this);
        arm.init();
        wrist.init();
        grabber.init();
        extender.init();
        distance.init();
        lift.init();
        droneLauncher.init();

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

        // Reset runtime
        runtime.reset();

        // Drop the arm down and reset arm angle
        dropArmAndReset();

        // Set initial state to intaking
        // After drop arm and reset, the arm now should be at intake position
        state = ArmState.intaking;

        // Main loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update current gamepad state
            curr1.copy(gamepad1);
            curr2.copy(gamepad2);

            // Control drivebase manually using joystick (field-oriented)
            drivebase.manualControl(true);

            // Check if these buttons are just newly pressed
            boolean grabberButton = curr2.cross && !prev2.cross;
            boolean armButton = curr2.dpad_left && !prev2.dpad_left;
            boolean quickResetButton = curr2.triangle && !prev2.triangle;

            // Arm finite state machine
            switch (state) {
                case none:
                    // Impossible state
                    break;
                case base_moving:
                    // This state will move the grabber up to avoid touching the ground

                    // Move grabber up -> no longer blocking drivebase's movement
                    wrist.setAngle(FULL_EXTEND_DEG);

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
                    }

                    break;
                case arm_moving_up:
                    // This state will move the arm up to outtake position

                    // If arm is extended -> retract it before moving
                    // Or the arm's PID will get crazy
                    if (extender.getPosition() > EXTENDER_FULLY_IN_THRESHOLD) {
                        extender.setPower(0.9);
                        isRetractExtenderTimeoutReset = true;
                    }

                    // When done retracing the arm's extender -> stop extender motor
                    else {
                        extender.setPower(0);
                        isRetractExtenderTimeoutReset = false;
                    }

                    // Move grabber up
                    wrist.setAngle(FULL_EXTEND_DEG);

                    // Use arm PID to move arm to desired angle
                    if (arm.anglePIDLoop() || armMoveUpTimeout.seconds() > 7) {
                        arm.setPower(0);
                        state = ArmState.outaking;
                    }

                    break;
                case intaking:
                    // This state will move grabber to intake position

                    // Set wrist angle to intake position (fully touch the ground)
                    wrist.setAngle(GROUND_PARALLEL_DEG);

                    // If grabber's reset button is pressed -> switch to quick_reset state
                    // Used to force the grabber to fully touch the ground
                    if (quickResetButton) {
                        armMoveDownTimeout.reset();
                        state = ArmState.quick_reset;
                    }

                    // If grabber button is pressed -> change state from base_moving to base_moving
                    // (Technically just move the grabber up to avoid touching the ground)
                    else if (grabberButton) {
                        state = ArmState.base_moving;
                    }

                    // If arm switch state button is pressed -> switch to arm_moving_up state
                    // (Technically moving arm up to outtake position)
                    else if (armButton) {
                        state = ArmState.arm_moving_up;
                        armMoveUpTimeout.reset();
                        arm.setTargetAngle(ARM_PARALLEL_ANGLE);
                        arm.resetPID();
                    }

                    break;
                case outaking:
                    // This state move grabber to outtake position

                    // Set grabber to auto parallel with backdrop
                    wrist.autoParallel(arm.getAngle());

                    // If arm switch state button is pressed -> switch to arm_moving_down state
                    // (Technically moving arm down to intake position)
                    if (armButton) {
                        armMoveDownTimeout.reset();
                        state = ArmState.arm_moving_down;
                    }

                    // Buttons for further tuning the arm's angle (just in case)
                    if (gamepad2.right_trigger > SENSE_TRIGGER) {
                        arm.setPower(0.15);
                    } else if (gamepad2.right_bumper) {
                        arm.setPower(-0.4);
                    } else {
                        arm.setPower(0);
                    }

                    break;
                case arm_moving_down:
                    // This state move the arm down to intake position

                    // If arm angle is < 45
                    if (arm.getAngle() < 45) {
                        // If just enter 45 degree state
                        if (!isRetractExtenderTimeoutReset) {
                            // Retract extender
                            retractExtenderTimeout.reset();
                            isRetractExtenderTimeoutReset = true;
                            extender.setPower(0.9);

                            // Move wrist to ground parallel position
                            wrist.setAngle(GROUND_PARALLEL_DEG);

                            // Hold the arm, waiting for extender and wrist to complete their job
                            arm.setPower(0);
                        }

                        // If extender is fully retracted
                        if (extender.getPosition() < EXTENDER_FULLY_IN_THRESHOLD || retractExtenderTimeout.seconds() > 1) {
                            extender.setPower(0);
                            extender.resetPosition();
                        }

                        // If done retracting
                        if (retractExtenderTimeout.seconds() > 1) {
                            arm.setPower(-0.1);
                        }
                    }

                    // If arm angle >= 45 -> set some speed to move the arm down
                    else {
                        arm.setPower(-0.6);
                    }

                    // If distance sensor reported touching ground or if arm is timeout
                    if (armMoveDownTimeout.seconds() > 5 || distance.getDistanceCM() <= DISTANCE_TO_GROUND_THRESHOLD) {
                        // Stop arm
                        arm.setPower(0);

                        // Reset arm encoder
                        arm.resetEncoder();

                        // Reset boolean
                        isRetractExtenderTimeoutReset = false;

                        // Switch to intaking state
                        state = ArmState.intaking;
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

            // Control left claw
            if (curr1.left_bumper && !prev1.left_bumper) {
                grablt = !grablt;
                grabber.leftClaw.setPosition(grablt ? 0.25 : 0);
            }

            // Control right claw
            if (curr1.right_bumper && !prev1.right_bumper) {
                grabrt = !grabrt;
                grabber.rightClaw.setPosition(grabrt ? 0.25 : 0);
            }

            // Launch drone
            if (curr1.circle && !curr1.circle) {
                if (droneLauncherShot) {
                    droneLauncher.setPosition(LOAD_DRONE_LAUNCHER_POSITION);
                    droneLauncherShot = false;
                } else {
                    droneLauncher.setPosition(SHOOT_DRONE_LAUNCHER_POSITION);
                    droneLauncherShot = true;
                }
            }

            // Hang
            if (gamepad1.triangle) {
                // Up
                lift.setPower(1);
            } else if (gamepad1.cross) {
                // Down
                lift.setPower(-1);
            } else {
                // Stop
                lift.setPower(0);
            }

            // Update prev1 gamepad
            prev1.copy(curr1);

            /**
             * General buttons mapping for gamepad2
             */

            // Control extender
            if (gamepad2.left_bumper) {
                extender.setPower(0.9);
            } else if (gamepad2.left_trigger > SENSE_TRIGGER) {
                extender.setPower(-0.9);
            } else if (!isRetractExtenderTimeoutReset) {
                extender.setPower(0);
            }

            // Update prev2 gamepad
            prev2.copy(curr2);

            // Show telemetry
            telemetry.addData("Current Arm Position (R)", arm.leftArmMotor.getCurrentPosition());
            telemetry.addData("Current Arm Angle (R)", arm.getAngle());
            telemetry.addData("Current Distance to Backdrop", distance.getDistanceCM());
            telemetry.addData("FSM State", state.toString());
            telemetry.addData("Extend position", extender.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
