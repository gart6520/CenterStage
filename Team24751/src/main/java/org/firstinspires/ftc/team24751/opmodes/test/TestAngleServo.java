package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.CAMERA_SERVO;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.LEFT_WRIST;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.RIGHT_WRIST;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.GOBILDA_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_ANGLE_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_PWM_RANGE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.team24751.subsystems.AngleServo;

import java.util.List;

/**
 * This opmode is used for testing the rotation of position-controlled servos
 * You can disable this opmode anytime you would like
 */

@TeleOp(name = "Test Angle Servo", group = "Test")
public class TestAngleServo extends LinearOpMode {
    // Servo object
    Gamepad curr, prev;

    @Override
    public void runOpMode() {
        // Init servo
        AngleServo angleServo = new AngleServo(CAMERA_SERVO, 0, 300, this);
        angleServo.init(GOBILDA_SERVO_PWM_RANGE);
        angleServo.getServo().setDirection(Servo.Direction.REVERSE);

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
        double rate = 45;
        curr = new Gamepad();
        curr.copy(gamepad1);
        prev = new Gamepad();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            curr.copy(gamepad1);
            if (curr.circle && !prev.circle)
            {
                angle += rate;
            }
            angleServo.setAngle(angle);
            telemetry.addData("Angle", angle);
            telemetry.addData("PWM", angleServo.getServo().getPosition());
            telemetry.update();
            prev.copy(curr);
        }
    }
}
