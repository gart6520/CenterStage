package org.firstinspires.ftc.team24751.opmodes.test;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.BACK_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.VISION.APRIL_TAG.INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_ANGLE_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_PWM_RANGE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;

import java.util.List;

@TeleOp(name = "Test Hand", group = "Test")
public class TestHand extends LinearOpMode {

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

        //Init camera
        Camera fieldCamera = new Camera(BACK_CAMERA_NAME, this);

        AngleServo angleServo = new AngleServo("servo", INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG, REV_SERVO_ANGLE_RANGE, this);
        angleServo.init(REV_SERVO_PWM_RANGE);
        angleServo.getServo().setDirection(Servo.Direction.REVERSE);
        angleServo.getServo().scaleRange(0.1, 0.9);

        // Wait for the driver to press PLAY
        waitForStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Loop, run until driver presses STOP
        double servoPos = 0;
        while (opModeIsActive()) {
            //Manual servo control
            if (gamepad1.dpad_right) {
                servoPos += 0.01;
                angleServo.getServo().setPosition(servoPos);
            } else if (gamepad1.dpad_left) {
                servoPos -= 0.01;
                angleServo.getServo().setPosition(servoPos);
            }
            servoPos = clamp(servoPos, 0 , 1);
            telemetry.addData("Servo angle", angleServo.getAngle());
            telemetry.addData("Servo PWM", angleServo.getServo().getPosition());
            telemetry.update();
//            Vector2d pos = teamProp.getCenterOfDetection();
//            telemetry.addData("Center of pixel", "\n" + pos.x + "\n" + pos.y);
        }
    }
}
