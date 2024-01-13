package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.INIT_WRIST_SERVO_ANGLE_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;

public class Wrist {
    LinearOpMode opMode;
    AngleServo leftWristServo;
    AngleServo rightWristServo;

    public Wrist(LinearOpMode _opMode) {
        opMode = _opMode;
        leftWristServo = new AngleServo(LEFT_WRIST, INIT_WRIST_SERVO_ANGLE_DEG, REV_SERVO_ANGLE_RANGE, opMode);
        rightWristServo = new AngleServo(RIGHT_WRIST, INIT_WRIST_SERVO_ANGLE_DEG, REV_SERVO_ANGLE_RANGE, opMode);
    }

    public void init() {
        leftWristServo.init(REV_SERVO_PWM_RANGE);
        leftWristServo.getServo().setDirection(Servo.Direction.FORWARD);
        rightWristServo.init(REV_SERVO_PWM_RANGE);
        rightWristServo.getServo().setDirection(Servo.Direction.REVERSE);
    }

    public void setSpeed(double speed) {
        leftWristServo.getServo().setPosition(leftWristServo.getServo().getPosition() + speed);
    }

    //Rotate the wrist parallel to the board (60 deg to the horizontal)
    public void autoParallel(double armAngleDeg) {
        double targetAngle = 0;
        if (armAngleDeg >= 120) {
            // Backdrop
            targetAngle = 342 - armAngleDeg;
        } else if (armAngleDeg <= 54) {
            // Ground
            targetAngle = 54 - armAngleDeg;
        } else {
            // Same as arm
            targetAngle = 60;
        }

        leftWristServo.setAngle(targetAngle);
        rightWristServo.setAngle(targetAngle);
        opMode.telemetry.addData("Wrist Target Angle", targetAngle);
        opMode.telemetry.addData("Wrist Pos", leftWristServo.getServo().getPosition());
    }
}
