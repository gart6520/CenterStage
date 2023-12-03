/*
 * Main manual drive OpMode for GART 24751's FTC code
 * Written by gvl610
 * Date created: 9/11/2023
 */

package org.firstinspires.ftc.team24751.opmodes.teleop;

// Import modules
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.GamepadHelper;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;

import static org.firstinspires.ftc.team24751.Constants.SPEED.*;

import java.util.List;

@TeleOp(name="Manual Control", group="Linear OpMode")
public class Manual extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystem objects
    private Gyro gyro = new Gyro(this);
    private Drivebase drivebase = new Drivebase(this, gyro);

    // Gamepad helper object
    GamepadHelper gp1 = new GamepadHelper(gamepad1);
    GamepadHelper gp2 = new GamepadHelper(gamepad2);

    // Mode flags
    private boolean manualDrivebase = true; // True if drivebase control mode is set to manual drive

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

        // Init subsystems
        gp1.init();
        gp2.init();
        gyro.init();
        drivebase.init();

        // Update gamepad values
        gp1.update();
        gp2.update();

        // Load last pose from auto mode
        drivebase.setCurrentPose(PoseStorage.getPose());

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();
        runtime.reset();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update gamepad values
            gp1.update();
            gp2.update();

            // Control drivebase manually if the manualDrivebase flag is true
            if (manualDrivebase) {
                // Get joystick axis values
                // Left joystick is used for driving bot in up/down/left/right direction, while right joystick is used for rotating the bot
                double left_y = -gamepad1.left_stick_y * DRIVEBASE_SPEED_Y; // Y axis is inverted
                double left_x = gamepad1.left_stick_x * DRIVEBASE_SPEED_X;
                double right_x = gamepad1.right_stick_x * DRIVEBASE_SPEED_Z;

                // Drive
                // drivebase.drive(left_x, left_y, right_x); // Drive bot-oriented
                drivebase.driveFieldOriented(left_x, left_y, right_x); // Drive field-oriented
            }

            // Button for resetting gyro's yaw
            if (gp1.options_just_pressed) {
                gyro.reset();
            }

            // TODO: Implement buttons for mechanisms and semi-auto drive
            // IMPORTANT NOTE: For semi-auto buttons, encoder MUST be reset
            // RUN_USING_ENCODER mode (using drivebase.resetRunUsingEncoder())
            // before following any trajectory, and MUST be reset back to
            // RUN_WITHOUT_ENCODER mode (using drivebase.resetRunWithoutEncoder())
            // after finishing the trajectory

            // Show elapsed run time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }
}
