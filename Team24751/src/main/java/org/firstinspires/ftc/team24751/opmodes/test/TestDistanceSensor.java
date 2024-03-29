package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

import java.util.List;

@TeleOp(name = "Test distance sensor", group = "Test")
public class TestDistanceSensor extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Drivebase drivebase = null;
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Extender extender = new Extender(this);

    // Gamepad
    Gamepad prev1 = null;
    Gamepad curr1 = null;
    Gamepad prev2 = null;
    Gamepad curr2 = null;
    boolean grablt = false;
    boolean grabrt = false;

    DistanceSensor distance = null;

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init all subsystems
        arm.init();


        wrist.init();
        grabber.init();
        extender.init();

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

        // Init drivebase
        drivebase = new Drivebase(this);

        // Load last pose from auto mode
        drivebase.setPoseEstimate(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        ElapsedTime timing = new ElapsedTime();
        timing.reset();
        while (timing.seconds() < 0.8) {
            arm.setPower(-0.5);
        }
        arm.setPower(0);
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
            double slow = 1;
            if (arm.getAngle() > 90)
            {
                slow = 0.5;
            }
            if (gamepad2.triangle) {
                arm.setPower(0.8 * slow);
            } else if (gamepad2.cross) {
                arm.setPower(-0.7 * slow);
            } else {
                arm.setPower(0);
            }

            if (gamepad2.dpad_down) {
                extender.setPower(0.6);
            } else if (gamepad2.dpad_up) {
                extender.setPower(-0.6);
            } else {
                extender.setPower(0);
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
            if (botVel.getX() != 0 || botVel.getY() != 0)
                drivebase.drive(botVel.getX(), botVel.getY(), 0);

            wrist.autoSetAngle(arm.getAngle());

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


            // Show pose estimation
            Pose2d pose = drivebase.getPoseEstimate();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getHeading());

//            telemetry.addData("Current Arm Position (R)", arm.rightArmMotor.getCurrentPosition());
//            telemetry.addData("Current Arm Angle (R)", arm.getAngle());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
