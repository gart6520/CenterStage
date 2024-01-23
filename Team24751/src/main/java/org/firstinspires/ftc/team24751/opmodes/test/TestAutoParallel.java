package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.MOTOR_POSITION_AT_FRONT_HORIZONTAL;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.MOTOR_POSITION_AT_UPWARD_VERTICAL;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

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

@TeleOp(name = "Test Auto //", group = "Test")
public class TestAutoParallel extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Drivebase drivebase = null;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Elevator elevator = new Elevator(this);
    Distance distance = new Distance(this);

    // Gamepad
    Gamepad prev = null;
    Gamepad curr = null;
    boolean grablt = false;
    boolean grabrt = false;

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

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update gamepad
            curr.copy(gamepad2);

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

            // TODO: Implement buttons for mechanisms and semi-auto drive
            // IMPORTANT NOTE: For semi-auto buttons, encoder MUST be reset
            // RUN_USING_ENCODER mode (using drivebase.resetRunUsingEncoder())
            // before following any trajectory, and MUST be reset back to
            // RUN_WITHOUT_ENCODER mode (using drivebase.resetRunWithoutEncoder())
            // after finishing the trajectory

            // Hitler Arm gogo
            if (gamepad2.triangle) {
                arm.setPower(0.6);
                arm.setTargetAngle(null);
            } else if (gamepad2.cross) {
                arm.setPower(-0.6);
                arm.setTargetAngle(null);
            } else {
                arm.setPower(0);
            }

            /*if (gamepad1.dpad_down)
            {
                arm.setTargetAngle(20.0);
                arm.resetPID();
            }
            else if (gamepad1.dpad_up)
            {
                arm.setTargetAngle(100.0);
                arm.resetPID();
            }*/

            if (gamepad2.dpad_left) {
                elevator.setPower(0.9);
            } else if (gamepad2.dpad_right) {
                elevator.setPower(-0.9);
            } else {
                elevator.setPower(0);
            }

            if (curr.dpad_up) {
                arm.distancePIDLoop(distance.getDistanceCM(), 3);
            }
            else {
                arm.resetPID();
            }

            if (curr.right_bumper)
            {
                wrist.setSpeed(0.01);
            }else if (curr.left_bumper)
            {
                wrist.setSpeed(-0.01);
            }
            if (curr.square && curr.square != prev.square) {
                grablt = !grablt;
                grabber.leftClaw.setPosition(grablt ? 1 : 0);
            }

            if (curr.circle && curr.circle != prev.circle) {
                grabrt = !grabrt;
                grabber.rightClaw.setPosition(grabrt ? 1 : 0);
            }

            // Update prev gamepad
            prev.copy(curr);

            // Show elapsed run time
            telemetry.addData("Current Arm Position (R)", arm.rightArmMotor.getCurrentPosition());
            telemetry.addData("Current Arm Angle (R)", arm.getAngle());
            telemetry.addData("Current Distance to Backdrop", distance.getDistanceCM());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
