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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;

import java.util.List;

@TeleOp(name = "Universal Test", group = "Test")
public class TestUniversal extends LinearOpMode {
    // Total run time
    private final ElapsedTime runtime = new ElapsedTime();
    /**
     * @noinspection FieldCanBeLocal
     */
    private final double maxMotorSpeed = 0.6;

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

        // Wait for the driver to press PLAY
        waitForStart();

        // Init gyro
        Gyro gyro = new Gyro(this);
        gyro.init();

        // Init drivebase
        Drivebase drivebase;
        try {
            drivebase = new Drivebase(this, gyro);
            drivebase.init();
        } catch (Exception e) {
            drivebase = null;
        }

        //Init all DCMotor and Servo
        DcMotorEx motor1 = hardwareMap.tryGet(DcMotorEx.class, "motor1");
        DcMotorEx motor2 = hardwareMap.tryGet(DcMotorEx.class, "motor2");
        DcMotorEx motor3 = hardwareMap.tryGet(DcMotorEx.class, "motor3");
        DcMotorEx motor4 = hardwareMap.tryGet(DcMotorEx.class, "motor4");

        Servo servo1 = hardwareMap.tryGet(Servo.class, "servo1");
        Servo servo2 = hardwareMap.tryGet(Servo.class, "servo2");
        Servo servo3 = hardwareMap.tryGet(Servo.class, "servo3");
        Servo servo4 = hardwareMap.tryGet(Servo.class, "servo4");

        CRServo crServo1 = hardwareMap.tryGet(CRServo.class, "servo1");
        CRServo crServo2 = hardwareMap.tryGet(CRServo.class, "servo2");
        CRServo crServo3 = hardwareMap.tryGet(CRServo.class, "servo3");
        CRServo crServo4 = hardwareMap.tryGet(CRServo.class, "servo4");
        // Load last pose from auto mode
        if (drivebase != null)
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
            double left_y = -gamepad1.left_stick_y * DRIVEBASE_SPEED_Y * speed; // Y axis is inverted
            double left_x = gamepad1.left_stick_x * DRIVEBASE_SPEED_X * speed;
            double right_x = gamepad1.right_stick_x * DRIVEBASE_SPEED_Z * speed;

            // Drive
            // drivebase.drive(left_x, left_y, right_x); // Drive bot-oriented
            if (drivebase != null)
                drivebase.driveFieldOriented(left_x, left_y, right_x); // Drive field-oriented

            //Motor and Servo control
            int invert = gamepad1.left_bumper ? -1 : 1;
            int servoInvert = gamepad1.left_bumper ? 0 : 1;

            if (motor1 != null)
                if (gamepad1.triangle) motor1.setPower(maxMotorSpeed * invert);
                else motor1.setPower(0);
            if (motor2 != null) {
                if (gamepad1.square) motor2.setPower(maxMotorSpeed * invert);
                else motor2.setPower(0);
            }
            if (motor3 != null) {
                if (gamepad1.cross) motor3.setPower(maxMotorSpeed * invert);
                else motor3.setPower(0);
            }
            if (motor4 != null) {
                if (gamepad1.circle) motor4.setPower(maxMotorSpeed * invert);
                else motor4.setPower(0);
            }

            if (gamepad1.dpad_up && servo1 != null) servo1.setPosition(servoInvert);
            if (gamepad1.dpad_left && servo2 != null) servo2.setPosition(servoInvert);
            if (gamepad1.dpad_down && servo3 != null) servo3.setPosition(servoInvert);
            if (gamepad1.dpad_right && servo4 != null) servo4.setPosition(servoInvert);

            if (crServo1 != null) {
                if (gamepad1.dpad_up) crServo1.setPower(invert);
                else crServo1.setPower(0);
            }
            if (crServo2 != null) {
                if (gamepad1.dpad_left) crServo2.setPower(invert);
                else crServo2.setPower(0);
            }
            if (crServo3 != null) {
                if (gamepad1.dpad_down) crServo3.setPower(invert);
                else crServo3.setPower(0);
            }
            if (crServo4 != null) {
                if (gamepad1.dpad_right) crServo4.setPower(invert);
                else crServo4.setPower(0);
            }
            telemetry.addData("Motor Power",
                    NullableMotorGetPower(motor1) + " " + NullableMotorGetPower(motor2) + " " +
                            NullableMotorGetPower(motor3) + " " + NullableMotorGetPower(motor4));
            telemetry.addData("Servo Position/Power",
                    NullableServoGet(servo1, crServo1), NullableServoGet(servo2, crServo2),
                    NullableServoGet(servo3, crServo3), NullableServoGet(servo4, crServo4));
            telemetry.update();
            // Show elapsed run time
            telemetry.addData("Status", "Run Time: " + runtime);
        }
    }

    private String NullableMotorGetPower(DcMotorEx motor) {
        if (motor != null) {
            return String.valueOf(motor.getPower());
        } else {
            return "";
        }
    }

    private String NullableServoGet(Servo servo, CRServo crServo) {
        if (servo != null) return String.valueOf(servo.getPosition());
        if (crServo != null) return String.valueOf(crServo.getPower());
        return "";
    }
}
