package org.firstinspires.ftc.team24751.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="Starter bot", group="Linear OpMode")
@Disabled
public class StarterBot extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Motors
    DcMotor leftBase = null;
    DcMotor rightBase = null;
    DcMotor leftElbow = null;
    DcMotor rightElbow = null;

    // Servos
    CRServo arm = null;
    CRServo grip = null;

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

        // Drivebase motors
        leftBase = hardwareMap.get(DcMotor.class, "leftBase");
        rightBase = hardwareMap.get(DcMotor.class, "rightBase");
        leftBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBase.setDirection(DcMotor.Direction.REVERSE);

        // Elbow motors
        leftElbow = hardwareMap.get(DcMotor.class, "leftElbow");
        rightElbow = hardwareMap.get(DcMotor.class, "rightElbow");
        leftElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElbow.setDirection(DcMotor.Direction.REVERSE);

        // Get servo
        arm = hardwareMap.get(CRServo.class, "arm");
        grip = hardwareMap.get(CRServo.class, "grip");

        // Wait for the driver to press PLAY
        waitForStart();

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset runtime
        runtime.reset();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Get button state
            boolean reverse = (gamepad1.left_trigger > 0.15);

            // Control drivebase manually
            final double speed = 0.9;
            leftBase.setPower(-gamepad1.left_stick_y * speed);
            rightBase.setPower(-gamepad1.right_stick_y * speed);

            // Elbow
            if (gamepad1.left_bumper) {
                leftElbow.setPower(reverse ? -0.3 : 0.3);
                rightElbow.setPower(reverse ? -0.3 : 0.3);
            } else {
                leftElbow.setPower(0);
                rightElbow.setPower(0);
            }

            // Arm
            if (gamepad1.right_bumper) {
                if (reverse) {
                    arm.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    arm.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                arm.setPower(0.6);
            } else {
                arm.setPower(0);
            }

            // Gripper
            if (gamepad1.right_trigger > 0.15) {
                if (reverse) {
                    grip.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    grip.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                grip.setPower(0.6);
            } else {
                grip.setPower(0);
            }

            // Show elapsed run time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }
}
