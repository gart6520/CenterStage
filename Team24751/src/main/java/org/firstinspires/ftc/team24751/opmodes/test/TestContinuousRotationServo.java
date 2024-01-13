package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GOBILDA_FAST_CRSERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.REV_SERVO_PWM_RANGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Test CR", group = "Test")
public class TestContinuousRotationServo extends LinearOpMode {
    CRServoImplEx servo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        servo = (CRServoImplEx)hardwareMap.get(CRServo.class, "servo");
        servo.setPwmRange(GOBILDA_FAST_CRSERVO_PWM_RANGE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                servo.setDirection(CRServoImplEx.Direction.FORWARD);
                servo.setPower(1);
            } else if (gamepad1.dpad_right) {
                servo.setDirection(CRServoImplEx.Direction.REVERSE);
                servo.setPower(1);
            } else servo.setPower(0);
        }
    }
}
