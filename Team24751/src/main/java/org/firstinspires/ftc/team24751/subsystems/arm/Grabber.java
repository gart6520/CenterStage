package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Grabber {
    LinearOpMode opMode;
    public ServoImplEx leftClaw;
    public ServoImplEx rightClaw;

    public Grabber(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        leftClaw = opMode.hardwareMap.get(ServoImplEx.class, LEFT_CLAW);
        leftClaw.setPwmRange(REV_SERVO_PWM_RANGE);

        rightClaw = opMode.hardwareMap.get(ServoImplEx.class, RIGHT_CLAW);
        rightClaw.setPwmRange(REV_SERVO_PWM_RANGE);
        rightClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(double left, double right) {
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }
}
