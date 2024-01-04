/*
 * Main manual drive OpMode for GART 24751's FTC code
 * Written by gvl610
 * Date created: 9/11/2023
 */

package org.firstinspires.ftc.team24751.opmodes.test;

// Import modules

import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;

import java.util.List;

@TeleOp(name="Test Intake Drivebase", group="Test")
public class TestIntakeDrivebase extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor intake;

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Init gyro
        Gyro gyro = new Gyro(this);
        gyro.init();

        // Init drivebase
        Drivebase drivebase = new Drivebase(this, gyro);
        drivebase.init();

        // Init intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Load last pose from auto mode
        drivebase.setCurrentPose(PoseStorage.getPose());

        // Wait for the driver to press PLAY
        waitForStart();

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset runtime
        runtime.reset();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
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

            if (gamepad1.left_bumper) {
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.9);
            } else if (gamepad1.right_bumper) {
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(0.9);
            } else {
                intake.setPower(0);
            }

            // Show elapsed run time
            telemetry.addData("Yaw", gyro.getYawDeg());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
