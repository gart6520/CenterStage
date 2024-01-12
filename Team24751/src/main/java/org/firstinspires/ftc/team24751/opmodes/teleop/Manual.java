package org.firstinspires.ftc.team24751.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Elevator;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;

import java.util.List;

@TeleOp(name = "Manual Hitler Arm", group = "!")
public class Manual extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Gyro gyro = new Gyro();
    Drivebase drivebase = new Drivebase();
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Elevator elevator = new Elevator(this);
    

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

        // Wait for the driver to press PLAY
        waitForStart();

        // Init gyro
        gyro.init(this);

        // Init drivebase
        drivebase.init(this, gyro);

        // Load last pose from auto mode
        drivebase.setCurrentPose(PoseStorage.getPose());

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
            double left_y = gamepad1.left_stick_y * DRIVEBASE_SPEED_Y * speed; // Y axis is inverted
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
            if (gamepad1.triangle) {
                arm.setPower(0.9);
            } else if (gamepad1.cross) {
                arm.setPower(0.8);
            } else {
                arm.setPower(0);
            }

            if (gamepad1.circle) {
                elevator.setPower(0.6);
            } else if (gamepad1.square) {
                elevator.setPower(-0.6);
            } else {
                elevator.setPower(0);
            }


            if (gamepad2.triangle) {
                wrist.setSpeed(0.01);
            } else if (gamepad2.cross) {
                wrist.setSpeed(-0.01);
            }

            if (gamepad2.square) {
                grabber.setPosition(1, 1);
            } else if (gamepad2.circle) {
                grabber.setPosition(0, 0);
            }

            // Show elapsed run time
            telemetry.addData("Yaw", gyro.getYawDeg());
            telemetry.addData("Current Arm Position (L-R)",
                    arm.leftArmMotor.getCurrentPosition() + " - " +
                            arm.rightArmMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
