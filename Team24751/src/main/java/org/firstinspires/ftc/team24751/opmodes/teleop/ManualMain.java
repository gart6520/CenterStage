package org.firstinspires.ftc.team24751.opmodes.teleop;

import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Elevator;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;

import java.util.List;

@TeleOp(name = "Main", group = "Manual")
public class ManualMain extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Gyro gyro = new Gyro();
    Drivebase drivebase = new Drivebase();
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Elevator elevator = new Elevator(this);

    // Gamepad
    Gamepad prev1 = null;
    Gamepad curr1 = null;
    Gamepad prev2 = null;
    Gamepad curr2 = null;
    boolean grablt = false;
    boolean grabrt = false;

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init all subsystems
        arm.init();
        arm.resetEncoder();

        wrist.init();
        grabber.init();
        elevator.init();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        prev1 = new Gamepad();
        prev1.copy(gamepad1);
        curr1 = new Gamepad();

        prev2 = new Gamepad();
        prev2.copy(gamepad2);
        curr2 = new Gamepad();

        // Init gyro
        gyro.init(this);

        // Init drivebase
        drivebase.init(this, gyro);

        // Load last pose from auto mode
        drivebase.setCurrentPose(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        ElapsedTime timing = new ElapsedTime();
        timing.reset();
        arm.resetEncoder();
        while (timing.seconds() < 0.8) {
            arm.setPower(-0.5);
        }
        arm.setPower(0);
        arm.resetEncoder();
        // Reset runtime
        runtime.reset();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update gamepad
            curr1.copy(gamepad1);
            curr2.copy(gamepad2);

            // Control drivebase manually
            // Get speed
            double speed = gamepad1.right_trigger > 0.15 ? 1 : 0.5;
            // Get joystick axis values
            // Left joystick is used for driving bot in up/down/left/right direction, while right joystick is used for rotating the bot
            double left_y = -gamepad1.left_stick_y * DRIVEBASE_SPEED_Y * speed; // Y axis is inverted
            double left_x = gamepad1.left_stick_x * DRIVEBASE_SPEED_X * speed;
            double right_x = gamepad1.right_stick_x * DRIVEBASE_SPEED_Z * speed;

            // Drive
//            drivebase.drive(left_x, left_y, right_x); // Drive bot-oriented
            drivebase.driveFieldOriented(left_x, left_y, right_x); // Drive field-oriented

            // TODO: Implement buttons for mechanisms and semi-auto drive
            // IMPORTANT NOTE: For semi-auto buttons, encoder MUST be reset
            // RUN_USING_ENCODER mode (using drivebase.resetRunUsingEncoder())
            // before following any trajectory, and MUST be reset back to
            // RUN_WITHOUT_ENCODER mode (using drivebase.resetRunWithoutEncoder())
            // after finishing the trajectory

            // Hitler Arm gogo
            if (gamepad2.triangle) {
                arm.setPower(0.8);
            } else if (gamepad2.cross) {
                arm.setPower(-0.7);
            } else {
                arm.setPower(0);
            }

            if (gamepad2.dpad_down) {
                elevator.setPower(0.6);
            } else if (gamepad2.dpad_up) {
                elevator.setPower(-0.6);
            } else {
                elevator.setPower(0);
            }
            Vector2d botVel = new Vector2d(0, 0);
            if (gamepad1.dpad_up) {
                botVel.plus(new Vector2d(0, 1));
            } else if (gamepad1.dpad_down) {
                botVel.plus(new Vector2d(0, -1));
            }
            if (gamepad1.dpad_right) {
                botVel.plus(new Vector2d(1, 0));
            } else if (gamepad1.dpad_left) {
                botVel.plus(new Vector2d(-1, 0));
            }
            if (botVel.x != 0 || botVel.y != 0)
                drivebase.drive(botVel.x, botVel.y, 0);

            wrist.autoParallel(arm.getAngle());

            if (curr2.circle && !prev2.circle) {
                wrist.isAuto = !wrist.isAuto;
            }
            if (curr1.left_bumper && curr1.left_bumper != prev1.left_bumper) {
                grablt = !grablt;
                grabber.leftClaw.setPosition(grablt ? 1 : 0);
            }

            if (curr1.right_bumper && curr1.right_bumper != prev1.right_bumper) {
                grabrt = !grabrt;
                grabber.rightClaw.setPosition(grabrt ? 1 : 0);
            }

            // Update prev gamepad
            prev1.copy(curr1);
            prev2.copy(curr2);


            // Show elapsed run time
            telemetry.addData("Yaw", gyro.getYawDeg());
            telemetry.addData("Current Arm Position (R)", arm.rightArmMotor.getCurrentPosition());
            telemetry.addData("Current Arm Angle (R)", arm.getAngle());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
