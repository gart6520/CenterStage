/*
 * Main manual drive OpMode for GART 24751's FTC code
 * Written by gvl610
 * Date created: 9/11/2023
 */

package org.firstinspires.ftc.team24751.opmodes.test;

// Import modules
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

import static org.firstinspires.ftc.team24751.Constants.SPEED.*;

import java.util.List;

@TeleOp(name="TestDrivebase", group="Test")
public class TestDrivebase extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystems
    private Drivebase drivebase = null;

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

        // Init drivebase
        drivebase = new Drivebase(this);

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
            drivebase.manualControl(true);

            // TODO: Implement buttons for mechanisms and semi-auto drive
            // IMPORTANT NOTE: For semi-auto buttons, encoder MUST be reset
            // RUN_USING_ENCODER mode (using drivebase.resetRunUsingEncoder())
            // before following any trajectory, and MUST be reset back to
            // RUN_WITHOUT_ENCODER mode (using drivebase.resetRunWithoutEncoder())
            // after finishing the trajectory

            // Show pose estimation
            Pose2d pose = drivebase.getPoseEstimate();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getHeading());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
