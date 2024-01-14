package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.ACTIVE_INTAKE_SERVO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ActiveIntake {
    private DcMotor activeIntakeServo;
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
        activeIntakeServo = opMode.hardwareMap.get(DcMotor.class, ACTIVE_INTAKE_SERVO);
    }

    public void run(IntakeDirection direction) {
        if (direction == IntakeDirection.REVERSE) {
            activeIntakeServo.setDirection(DcMotor.Direction.REVERSE);
            activeIntakeServo.setPower(1);
        }
        if (direction == IntakeDirection.FORWARD) {
            activeIntakeServo.setDirection(DcMotor.Direction.FORWARD);
            activeIntakeServo.setPower(1);
        }
        if (direction == IntakeDirection.STATIONARY) {
            activeIntakeServo.setPower(0);
        }
    }
}
