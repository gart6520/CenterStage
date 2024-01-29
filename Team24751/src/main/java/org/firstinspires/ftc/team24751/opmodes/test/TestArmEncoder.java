package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Utility.enableBulkRead;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

@TeleOp(name = "Test Arm Encoder", group = "Test")
public class TestArmEncoder extends LinearOpMode {
    Drivebase drivebase = null;
    Arm arm = new Arm(this);
    // Gamepad
    Gamepad prev = null;
    Gamepad curr = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init all subsystems
        drivebase = new Drivebase(this);
        arm.init();


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
        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Update gamepad
            curr.copy(gamepad2);

            // Control drivebase manually
            drivebase.manualControl(false);


            if (curr.circle && !prev.circle) {
                arm.resetEncoder();
            }

            // Update prev gamepad
            prev.copy(curr);

            // Show telemetry
            telemetry.addData("Current Arm Position (R)", arm.rightArmMotor.getCurrentPosition());
            telemetry.addData("Current Arm Angle (R)", arm.getAngle());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
