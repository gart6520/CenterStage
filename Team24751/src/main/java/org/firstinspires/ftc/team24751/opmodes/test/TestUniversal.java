/*
 * Main manual drive OpMode for GART 24751's FTC code
 * Written by gvl610
 * Date created: 9/11/2023
 */

package org.firstinspires.ftc.team24751.opmodes.test;

// Import modules

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GOBILDA_FAST_CRSERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GOBILDA_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.REV_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
    private final double maxMotorSpeed = 0.4;

    private Gyro gyro = new Gyro();

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
        gyro.init(this);

        // Init drivebase
        Drivebase drivebase;
        try {
            drivebase = new Drivebase();
            drivebase.init(this, gyro);
        } catch (Exception e) {
            drivebase = null;
        }

        //Init all DCMotor and Servo
        DcMotorEx motor1 = hardwareMap.tryGet(DcMotorEx.class, "motor1");
        DcMotorEx motor2 = hardwareMap.tryGet(DcMotorEx.class, "motor2");
        DcMotorEx motor3 = hardwareMap.tryGet(DcMotorEx.class, "motor3");
        DcMotorEx motor4 = hardwareMap.tryGet(DcMotorEx.class, "motor4");

        ServoImplEx servo1 = hardwareMap.tryGet(ServoImplEx.class, "servo1");
        if (servo1 != null) servo1.setPwmRange(REV_SERVO_PWM_RANGE);
        ServoImplEx servo2 = hardwareMap.tryGet(ServoImplEx.class, "servo2");
        if (servo2 != null) servo2.setPwmRange(REV_SERVO_PWM_RANGE);
        ServoImplEx servo3 = hardwareMap.tryGet(ServoImplEx.class, "servo3");
        if (servo3 != null) servo3.setPwmRange(GOBILDA_SERVO_PWM_RANGE);
        ServoImplEx servo4 = hardwareMap.tryGet(ServoImplEx.class, "servo4");
        if (servo4 != null) servo4.setPwmRange(GOBILDA_SERVO_PWM_RANGE);

        CRServoImplEx crServo1 = hardwareMap.tryGet(CRServoImplEx.class, "servo1");
        if (crServo1 != null) crServo1.setPwmRange(GOBILDA_FAST_CRSERVO_PWM_RANGE);
        CRServoImplEx crServo2 = hardwareMap.tryGet(CRServoImplEx.class, "servo2");
        if (crServo2 != null) crServo2.setPwmRange(GOBILDA_FAST_CRSERVO_PWM_RANGE);
        CRServoImplEx crServo3 = hardwareMap.tryGet(CRServoImplEx.class, "servo3");
        if (crServo3 != null) crServo3.setPwmRange(GOBILDA_FAST_CRSERVO_PWM_RANGE);
        CRServoImplEx crServo4 = hardwareMap.tryGet(CRServoImplEx.class, "servo4");
        if (crServo4 != null) crServo4.setPwmRange(GOBILDA_FAST_CRSERVO_PWM_RANGE);
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
