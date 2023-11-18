/*
 * Main class for GART 24751's FTC code
 * Written by gvl610
 * Date created: 9/11/2023
 */

package org.firstinspires.ftc.team24751.opmodes;

// Import modules
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import static org.firstinspires.ftc.team24751.Constants.SPEED.*;

import java.util.List;

@TeleOp(name="Manual Control", group="Linear OpMode")
public class Manual extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystem objects
    private Gyro gyro = new Gyro(this);
    private Drivebase drivebase = new Drivebase(this, gyro);

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
        gyro.init();
        drivebase.init();

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();
        runtime.reset();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Get joystick axis values
            // Left joystick is used for driving bot in up/down/left/right direction, while right joystick is used for rotating the bot
            double left_y = -gamepad1.left_stick_y * DRIVEBASE_SPEED_Y; // Y axis is inverted
            double left_x = gamepad1.left_stick_x * DRIVEBASE_SPEED_X;
            double right_x = gamepad1.right_stick_x * DRIVEBASE_SPEED_Z;

            // Drive
            drivebase.drive(left_x, left_y, right_x);

            // Show elapsed run time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }
}
