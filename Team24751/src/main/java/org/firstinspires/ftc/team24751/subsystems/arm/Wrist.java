package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.WRIST_SERVO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GOBILDA_SERVO_ANGLE_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GOBILDA_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.INIT_WRIST_SERVO_ANGLE_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.REV_SERVO_PWM_RANGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;

public class Wrist {
    LinearOpMode opMode;
    AngleServo wristServo;

    public Wrist(LinearOpMode _opMode) {
        opMode = _opMode;
        wristServo = new AngleServo(WRIST_SERVO, INIT_WRIST_SERVO_ANGLE_DEG, GOBILDA_SERVO_ANGLE_RANGE, opMode);
    }

    public void init() {
        wristServo.init(REV_SERVO_PWM_RANGE);
        wristServo.getServo().setDirection(Servo.Direction.FORWARD);
    }

    public void setSpeed(double speed) {
        wristServo.getServo().setPosition(wristServo.getServo().getPosition() + speed);
    }

    //Rotate the wrist parallel to the board (60 deg to the horizontal)
    public void autoParallel(double armAngleDeg) {
        double targetAngle = 0;
        if (armAngleDeg > 90) {
            // Backdrop
            targetAngle = armAngleDeg + 120;
        } else {
            // Ground
            targetAngle = armAngleDeg + 60;
        }

        wristServo.setAngle(targetAngle);
        opMode.telemetry.addData("Wrist Target Angle", targetAngle);
        opMode.telemetry.addData("Wrist Pos", wristServo.getServo().getPosition());
    }
}
