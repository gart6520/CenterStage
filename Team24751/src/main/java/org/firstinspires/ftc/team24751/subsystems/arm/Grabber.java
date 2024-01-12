package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.LEFT_CLAW;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.RIGHT_CLAW;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Grabber {
    LinearOpMode opMode;
    ServoImplEx leftClaw;
    ServoImplEx rightClaw;

    public Grabber(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        leftClaw = opMode.hardwareMap.get(ServoImplEx.class, LEFT_CLAW);
        rightClaw = opMode.hardwareMap.get(ServoImplEx.class, RIGHT_CLAW);
    }

    public void setPosition(double left, double right) {
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }
}
