package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.GOBILDA_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_ANGLE_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_PWM_RANGE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.subsystems.AngleServo;

import java.util.List;

/**
 * This opmode is used for testing the rotation of position-controlled servos
 * You can disable this opmode anytime you would like
 */

@TeleOp(name = "TestServo", group = "Test")
public class TestServo extends LinearOpMode {
    // Servo object
    ServoImplEx servo = null;

    @Override
    public void runOpMode() {
        // Init servo
        AngleServo leftAngleServo = new AngleServo(LEFT_WRIST, 0, REV_SERVO_ANGLE_RANGE, this);
        AngleServo rightAngleServo = new AngleServo(RIGHT_WRIST, 0, REV_SERVO_ANGLE_RANGE, this);
        ServoImplEx tuneServo = hardwareMap.get(ServoImplEx.class, YELLOW_PIXEL_YEETER);
        leftAngleServo.init(REV_SERVO_PWM_RANGE);
        rightAngleServo.init(REV_SERVO_PWM_RANGE);
        tuneServo.setPwmRange(GOBILDA_SERVO_PWM_RANGE);

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the driver to press PLAY
        waitForStart();

        // Init angle
        // Angle range is from 0 -> 300
        double angle = 0;
        double rate = 1;

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            // Display current target angle


            // Set angle
            //servo.setPosition(angle/270.0);
            //servo.setPosition(0);

            // Overflow
            //if (angle >= 270) {
            //    rate = -1;
            //} else if (angle <= 0) {
            //    rate = 1;
            //}

            // Increase by rate
            //angle += rate;

            // Delay 50ms
            //sleep(50);
            if (gamepad1.dpad_left) {
                leftAngleServo.setAngle(0);
                rightAngleServo.setAngle(270);
            }

            if (gamepad1.dpad_right) {
                leftAngleServo.setAngle(270);
                rightAngleServo.setAngle(0);
            }
            if (gamepad1.dpad_up) {
                tuneServo.setPosition(0);
            }
            if (gamepad1.dpad_down) {
                tuneServo.setPosition(1);
            }

            telemetry.update();
        }
    }
}
