package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.ClimberHolder.HOLD_CLIMBER_HOLDER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.ClimberHolder.RELEASE_CLIMBER_HOLDER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.GOBILDA_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_ANGLE_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.YellowPixelYeeter.LOAD_YELLOW_PIXEL_YEETER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.YellowPixelYeeter.YEET_YELLOW_PIXEL_YEETER_POSITION;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;
import org.firstinspires.ftc.team24751.subsystems.ClimberHolder;
import org.firstinspires.ftc.team24751.subsystems.YellowPixelYeeter;

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
        YellowPixelYeeter yellowPixelYeeter = new YellowPixelYeeter(this);
        ClimberHolder climberHolder = new ClimberHolder(this);
        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the driver to press PLAY
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                yellowPixelYeeter.setPosition(LOAD_YELLOW_PIXEL_YEETER_POSITION);
            }
            if (gamepad1.dpad_down) {
                yellowPixelYeeter.setPosition(YEET_YELLOW_PIXEL_YEETER_POSITION);
            }

            if (gamepad1.dpad_left)
            {
                climberHolder.setPosition(HOLD_CLIMBER_HOLDER_POSITION);
            }
            if (gamepad1.dpad_right)
            {
                climberHolder.setPosition(RELEASE_CLIMBER_HOLDER_POSITION);
            }

            telemetry.update();
        }
    }
}
