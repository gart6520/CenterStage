package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.ACTIVE_INTAKE_SERVO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

public class ActiveIntake {
    private CRServoImplEx activeIntakeServo;
    private LinearOpMode opMode;

    public enum IntakeDirection {
        FORWARD,
        REVERSE,
        STATIONARY
    }

    public ActiveIntake(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        activeIntakeServo = opMode.hardwareMap.get(CRServoImplEx.class, ACTIVE_INTAKE_SERVO);
    }

    public void run(IntakeDirection direction) {
        if (direction == IntakeDirection.REVERSE) {
            activeIntakeServo.setDirection(CRServoImplEx.Direction.REVERSE);
            activeIntakeServo.setPower(1);
        }
        if (direction == IntakeDirection.FORWARD) {
            activeIntakeServo.setDirection(CRServoImplEx.Direction.FORWARD);
            activeIntakeServo.setPower(1);
        }
        if (direction == IntakeDirection.STATIONARY) {
            activeIntakeServo.setPower(0);
        }
    }
}
