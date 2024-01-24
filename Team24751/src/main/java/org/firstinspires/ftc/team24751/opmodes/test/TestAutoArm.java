package org.firstinspires.ftc.team24751.opmodes.test;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Distance;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Elevator;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

import java.util.List;
import java.util.function.BooleanSupplier;

@TeleOp(name = "Test Auto Arm", group = "Test")
public class TestAutoArm extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Drivebase drivebase = null;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Elevator elevator = new Elevator(this);
    Distance distance = new Distance(this);

    enum ArmState {
        none, base_moving, arm_moving_up, arm_moving_down, intaking, outaking, quick_reset
    }

    ArmState state = ArmState.none;
    // Gamepad
    Gamepad prev = null;
    Gamepad curr = null;
    boolean grablt = false;
    boolean grabrt = false;
    ElapsedTime armMoveDownTimer = new ElapsedTime();


    private void dropArmAndReset() {
        ElapsedTime timer = new ElapsedTime();
        wrist.setAngle(43);
        timer.reset();
        while (timer.seconds() > 0.3 && timer.seconds() < 1.3) {
            arm.setPower(-0.5);
        }
        arm.setPower(0);
        arm.resetEncoder();
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
        elevator.init();
        distance.init();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
            drivebase.manualControl(true);


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
            final double armParallelAngle = 141;
            // Finite state machine
//            arm.update();
            switch (state) {
                case none:
                    // Impossible state
                    break;
                case base_moving:
                    // Ensure grabber is not in the way
                    wrist.isAuto = false;
                    wrist.autoParallel(arm.getAngle());
                    // Grabber into intake position
                    if (grabberButton.getAsBoolean()) {
                        wrist.isAuto = true;
                        state = ArmState.intaking;
                    }
                    // Arm up for outaking
                    else if (armButton.getAsBoolean()) {
                        wrist.isAuto = true;
                        state = ArmState.arm_moving_up;
                        arm.setTargetAngle(armParallelAngle);
                        arm.resetPID();
                    }
                    break;
                case arm_moving_up:
                    // Get grabber out of the way
                    wrist.setAngle(250);
                    // PID
                    if (arm.anglePIDLoop()) {
                        state = ArmState.outaking;
                    }
                    break;
                case intaking:
                    // Reset grabber when misaligned
                    wrist.autoParallel(arm.getAngle());
                    if (quickResetButton.getAsBoolean()) {
                        armMoveDownTimer.reset();
                        state = ArmState.quick_reset;
                    } // Get grabber out of the way
                    else if (grabberButton.getAsBoolean()) {
                        state = ArmState.base_moving;
                    } // Arm up for outaking
                    else if (armButton.getAsBoolean()) {
                        state = ArmState.arm_moving_up;
                        arm.setTargetAngle(armParallelAngle);
                        arm.resetPID();
                    }
                    break;
                case outaking:
                    // Grabber auto parallel board
                    wrist.autoParallel(arm.getAngle());
                    // Move arm down
                    if (armButton.getAsBoolean()) {
                        armMoveDownTimer.reset();
                        state = ArmState.arm_moving_down;
                    }
                    break;
                case arm_moving_down:
                    wrist.autoParallel(arm.getAngle());
                    // Move down until timer
                    arm.setPower(-0.6);
                    if (armMoveDownTimer.seconds() > 3) {
                        arm.resetEncoder();
                        arm.setPower(0);
                        state = ArmState.intaking;
                    }
                    break;
                case quick_reset:
                    // Initially move up
                    if (armMoveDownTimer.seconds() < 0.5) {
                        arm.setPower(0.5);
                    } //Then move down
                    else if (armMoveDownTimer.seconds() < 1.25) {
                        arm.setPower(-0.4);
                    } else {
                        arm.resetEncoder();
                        state = ArmState.intaking;
                    }
                    break;
            }

            if (gamepad2.dpad_left) {
                elevator.setPower(0.9);
            } else if (gamepad2.dpad_right) {
                elevator.setPower(-0.9);
            } else {
                elevator.setPower(0);
            }


            if (curr.square && !prev.square) {
                grablt = !grablt;
                grabber.leftClaw.setPosition(grablt ? 1 : 0);
            }

            if (curr.circle && !prev.circle) {
                grabrt = !grabrt;
                grabber.rightClaw.setPosition(grabrt ? 1 : 0);
            }

            // Update prev gamepad
            prev.copy(curr);

            // Show elapsed run time
            telemetry.addData("Current Arm Position (R)", arm.rightArmMotor.getCurrentPosition());
            telemetry.addData("Current Arm Angle (R)", arm.getAngle());
            telemetry.addData("Current Distance to Backdrop", distance.getDistanceCM());
            telemetry.addData("FSM State", state.toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
