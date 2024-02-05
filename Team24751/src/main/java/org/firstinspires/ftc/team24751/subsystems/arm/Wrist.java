package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_BACKDROP_PARALLEL_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.INIT_WRIST_SERVO_ANGLE_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;

public class Wrist {
    LinearOpMode opMode;
    AngleServo leftWristServo;
    AngleServo rightWristServo;
    public boolean isAuto = true;

    public Wrist(LinearOpMode _opMode) {
        opMode = _opMode;
        leftWristServo = new AngleServo(
                LEFT_WRIST, INIT_WRIST_SERVO_ANGLE_DEG, GENERAL_SERVO.REV_SERVO_ANGLE_RANGE, opMode);
        rightWristServo = new AngleServo(
                RIGHT_WRIST, INIT_WRIST_SERVO_ANGLE_DEG, GENERAL_SERVO.REV_SERVO_ANGLE_RANGE, opMode);
    }

    public void init() {
        leftWristServo.init(GENERAL_SERVO.REV_SERVO_PWM_RANGE);
        leftWristServo.getServo().setDirection(Servo.Direction.REVERSE);
        rightWristServo.init(GENERAL_SERVO.REV_SERVO_PWM_RANGE);
        rightWristServo.getServo().setDirection(Servo.Direction.FORWARD);
    }

    public void setSpeed(double speed) {
        leftWristServo.getServo().setPosition(leftWristServo.getServo().getPosition() + speed);
    }
    public void setAngle (double angle)
    {
        leftWristServo.setAngle(angle);
        rightWristServo.setAngle(angle);
    }
    /**
     * Automatically rotate the grabber in case the arm need to be operated manually
     * */
    public void autoSetAngle(double armAngleDeg) {
        double targetAngle;
        if (isAuto) {
            if (armAngleDeg >= 90) {
                // Backdrop
                targetAngle = 348 - armAngleDeg;
            } else if (armAngleDeg <= 52) {
                // Ground
                targetAngle = 43;
            } else {
                // Same as arm
                targetAngle = 60;
            }
        } else {
            // Arm right up in base moving
            targetAngle = 150;
        }

        setAngle(targetAngle);
        opMode.telemetry.addData("Wrist Target Angle", targetAngle);
        opMode.telemetry.addData("Wrist Pos", leftWristServo.getServo().getPosition());
    }

    public void autoParallel(double armAngleDeg)
    {
        setAngle(armAngleDeg - WRIST_BACKDROP_PARALLEL_DEG);
    }
}
