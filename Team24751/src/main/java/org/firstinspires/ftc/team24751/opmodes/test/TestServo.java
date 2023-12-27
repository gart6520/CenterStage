package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.List;

/**
 * This opmode is used for testing the rotation of position-controlled servos
 * You can disable this opmode anytime you would like
 */

@TeleOp(name="TestServo", group="Test")
public class TestServo extends LinearOpMode {
    // Servo object
    ServoImplEx servo = null;

    @Override
    public void runOpMode() {
        // Init servo
        servo = hardwareMap.get(ServoImplEx.class, "servo");
        servo.setPwmRange(new PwmControl.PwmRange(550, 2450));
        servo.setDirection(ServoImplEx.Direction.REVERSE);

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
                servo.setPosition(0);
            }

            if (gamepad1.dpad_right) {
                servo.setPosition(1);
            }

            telemetry.update();
        }
    }
}
