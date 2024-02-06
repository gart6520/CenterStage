package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.CAMERA_SERVO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.GOBILDA_SERVO_PWM_RANGE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;
import org.firstinspires.ftc.team24751.subsystems.LedIndicator;

import java.util.List;

/**
 * This opmode is used for testing the rotation of position-controlled servos
 * You can disable this opmode anytime you would like
 */

@TeleOp(name = "Test Led Indicator", group = "Test")
public class TestLedIndicator extends LinearOpMode {
    // Servo object
    Gamepad curr, prev;
    boolean red = false, green = false;

    @Override
    public void runOpMode() {
        // Init servo
        LedIndicator led = new LedIndicator(this);
        led.init();
        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the driver to press PLAY
        waitForStart();

        // Init angle
        // Angle range is from 0 -> 300
        curr = new Gamepad();
        curr.copy(gamepad1);
        prev = new Gamepad();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            curr.copy(gamepad1);
            if (curr.circle && !prev.circle)
            {
                red = !red;
            }
            if (curr.square && !prev.square)
            {
                green = !green;
            }
            led.setColor(red, green);
            telemetry.addData("Led state (R G)", red + " " + green);
            telemetry.update();
            prev.copy(curr);
        }
    }
}
