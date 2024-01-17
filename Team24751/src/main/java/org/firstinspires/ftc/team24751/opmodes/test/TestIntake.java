/*
 * Main manual drive OpMode for GART 24751's FTC code
 * Written by gvl610
 * Date created: 9/11/2023
 */

package org.firstinspires.ftc.team24751.opmodes.test;

// Import modules

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="Test Intake", group="Test")
public class TestIntake extends LinearOpMode {
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

        // Init intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Load last pose from auto mode

        // Wait for the driver to press PLAY
        waitForStart();

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset runtime
        runtime.reset();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.9);
            } else if (gamepad1.right_bumper) {
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(0.9);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}