package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_PARALLEL_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.DISTANCE_TO_GROUND_THRESHOLD;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.FULL_EXTEND_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.GROUND_PARALLEL_DEG;
import static org.firstinspires.ftc.team24751.Utility.enableBulkRead;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Distance;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Test Auto Arm", group = "Test")
public class TestAutoArm extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Drivebase drivebase = null;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Extender extender = new Extender(this);
    Distance distance = new Distance(this);
    AHRS navx = null;

    enum ArmState {
        none, base_moving, arm_moving_up, arm_moving_down, intaking, outaking, quick_reset
    }

    ArmState state = ArmState.none;
    // Gamepad
    Gamepad prev = null;
    Gamepad curr = null;
    boolean grablt = false;
    boolean grabrt = false;
    ElapsedTime armMoveDownTimeout = new ElapsedTime();
    ElapsedTime armMoveUpTimeout = new ElapsedTime();
    ElapsedTime distSensorNailInTheCoffinTimer = new ElapsedTime();
    ElapsedTime undoExtendTimeout = new ElapsedTime();
    boolean isUndoExtendTimeoutReset = false;


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
        arm.setPower(0);
        arm.resetEncoder();
//        navx.zeroYaw();
    }

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init all subsystems
        drivebase = new Drivebase(this);
        arm.init();
        wrist.init();
        grabber.init();
        extender.init();
        distance.init();
//        navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
//                AHRS.DeviceDataType.kProcessedData);

        // Enable bulk reads in auto mode
        enableBulkRead(hardwareMap);

        prev = new Gamepad();
        prev.copy(gamepad2);
        curr = new Gamepad();

        // Load last pose from auto mode
        drivebase.setPoseEstimate(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // Reset runtime
        runtime.reset();
        dropArmAndReset();
        state = ArmState.intaking;
        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update gamepad
            curr.copy(gamepad2);

            // Control drivebase manually
            drivebase.manualControl(false);


            /*if (gamepad1.dpad_down)
            {
                arm.setTargetAngle(20.0);
                arm.resetPID();
            }
            else if (gamepad1.dpad_up)
            {
                arm.setTargetAngle(100.0);
                arm.resetPID();
            }
            if (curr.dpad_up) {
                arm.distancePIDLoop(distance.getDistanceCM(), 3);
            }
            else {
                arm.resetPID();
            }*/
            BooleanSupplier grabberButton = () -> curr.cross && !prev.cross;
            BooleanSupplier armButton = () -> curr.triangle && !prev.triangle;
            BooleanSupplier quickResetButton = () -> curr.left_bumper && !prev.left_bumper;
            // Finite state machine
//            arm.update();
            switch (state) {
                case none:
                    // Impossible state
                    break;
                case base_moving:
                    // Ensure grabber is not in the way
                    wrist.setAngle(FULL_EXTEND_DEG);
                    // Grabber into intake position
                    if (grabberButton.getAsBoolean()) {
                        state = ArmState.intaking;
                    }
                    // Arm up for outaking
                    else if (armButton.getAsBoolean()) {
                        state = ArmState.arm_moving_up;
                        armMoveUpTimeout.reset();
                        arm.setTargetAngle(ARM_PARALLEL_ANGLE);
                        arm.resetPID();
                    }
                    break;
                case arm_moving_up:
                    if (extender.getPosition() > 50)
                    {
                        extender.setPower(0.9);
                        isUndoExtendTimeoutReset = true;
                    }
                    else {
                        extender.setPower(0);
                        isUndoExtendTimeoutReset = false;
                    }
                    // Get grabber out of the way
                    wrist.setAngle(FULL_EXTEND_DEG);
                    // PID
                    if (arm.anglePIDLoop() || armMoveUpTimeout.seconds() > 7) {
                        arm.setPower(0);
                        state = ArmState.outaking;
                    }
                    break;
                case intaking:
                    // Reset grabber when misaligned
                    wrist.setAngle(GROUND_PARALLEL_DEG);
                    if (quickResetButton.getAsBoolean()) {
                        armMoveDownTimeout.reset();
                        state = ArmState.quick_reset;
                    } // Get grabber out of the way
                    else if (grabberButton.getAsBoolean()) {
                        state = ArmState.base_moving;
                    } // Arm up for outaking
                    else if (armButton.getAsBoolean()) {
                        state = ArmState.arm_moving_up;
                        armMoveUpTimeout.reset();
                        arm.setTargetAngle(ARM_PARALLEL_ANGLE);
                        arm.resetPID();
                    }
                    break;
                case outaking:
                    // Grabber auto parallel board
                    wrist.autoParallel(arm.getAngle());
                    // Move arm down
                    if (armButton.getAsBoolean()) {
                        armMoveDownTimeout.reset();
                        state = ArmState.arm_moving_down;
                    }
                    if (gamepad2.dpad_up) {
                        arm.setPower(0.15);
                    } else if (gamepad2.dpad_down) {
                        arm.setPower(-0.15);
                    } else {
                        arm.setPower(0);
                    }
                    break;
                case arm_moving_down:
                    // Move down until timer
                    if (arm.getAngle() < 90) {
                        if (!isUndoExtendTimeoutReset) {
                            undoExtendTimeout.reset();
                            isUndoExtendTimeoutReset = true;
                            wrist.setAngle(GROUND_PARALLEL_DEG);
                            extender.setPower(0.9);
                            arm.setPower(0);
                        }

                        // Undo extend
                        if (extender.getPosition() < 50 || undoExtendTimeout.seconds() > 1) {
                            extender.setPower(0);
                            extender.resetPosition();
                        }

                        if (undoExtendTimeout.seconds() > 1) {
                            arm.setPower(-0.1);
                        }
                    } else {
                        arm.setPower(-0.6);
                    }

                    //Go until distance sensor report // to the ground or timeout
                    if (armMoveDownTimeout.seconds() > 5 || distance.getDistanceCM() <= DISTANCE_TO_GROUND_THRESHOLD) {
                        arm.resetEncoder();
                        arm.setPower(0);
                        isUndoExtendTimeoutReset = false;
                        state = ArmState.intaking;
                    }
                    break;
                case quick_reset:
                    wrist.setAngle(GROUND_PARALLEL_DEG);
                    // Initially move up
                    if (armMoveDownTimeout.seconds() < 0.5) {
                        arm.setPower(0.5);
                    } //Then move down
                    else if (armMoveDownTimeout.seconds() < 3 && distance.getDistanceCM() >= DISTANCE_TO_GROUND_THRESHOLD) {
                        arm.setPower(-0.4);
                    } else {
                        arm.resetEncoder();
                        arm.setPower(0);
                        state = ArmState.intaking;
                    }
                    break;
            }

            if (gamepad2.dpad_left) {
                extender.setPower(0.9);
            } else if (gamepad2.dpad_right) {
                extender.setPower(-0.9);
            } else if (!isUndoExtendTimeoutReset) {
                extender.setPower(0);
            }


            if (curr.circle && !prev.circle) {
                grablt = !grablt;
                grabber.leftClaw.setPosition(grablt ? 0.25 : 0);
            }

            if (curr.square && !prev.square) {
                grabrt = !grabrt;
                grabber.rightClaw.setPosition(grabrt ? 0.25 : 0);
            }

            // Update prev gamepad
            prev.copy(curr);

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
