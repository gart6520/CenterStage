package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;

import java.util.List;

@TeleOp(name="TestAllManual", group="Test")
public class TestAllManual extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Gyro gyro = new Gyro();
    Drivebase drivebase = new Drivebase();

    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;
    private DcMotor elevatorMotor = null;
    private CRServo wrist = null;
    private Servo leftClaw = null;

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        wrist = hardwareMap.get(CRServo.class, "wristServo");
        leftClaw = hardwareMap.get(Servo.class, "leftClawServo");

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                leftArmMotor.setPower(0.9);
                rightArmMotor.setPower(0.9);
            } else if (gamepad1.cross) {
                leftArmMotor.setPower(-0.8);
                rightArmMotor.setPower(-0.8);
            } else {
                leftArmMotor.setPower(0);
                rightArmMotor.setPower(0);
            }

            if (gamepad1.circle) {
                elevatorMotor.setPower(0.6);
            } else if (gamepad1.square) {
                elevatorMotor.setPower(-0.6);
            } else {
                elevatorMotor.setPower(0);
            }


            if (gamepad2.triangle) {
                wrist.setDirection(CRServo.Direction.FORWARD);
                wrist.setPower(0.3);
            } else if (gamepad2.cross) {
                wrist.setDirection(CRServo.Direction.REVERSE);
                wrist.setPower(0.3);
            } else {
                wrist.setPower(0);
            }

            if (gamepad2.square) {
                leftClaw.setPosition(1);
            } else if (gamepad2.circle) {
                leftClaw.setPosition(0);
            }

            // Show elapsed run time
            telemetry.addData("Yaw", gyro.getYawDeg());
            telemetry.addData("Current Arm Position", leftArmMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
