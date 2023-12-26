package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.SENSITIVITY.SERVO_ANGLE_PWM_THRESHOLD;
import static org.firstinspires.ftc.team24751.Constants.SENSITIVITY.SERVO_PWM_SPEED;
import static org.firstinspires.ftc.team24751.Utility.wrapAngle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.Utility;

/**
 * Servo wrapper that handle get/set angle instead of PWM
 */
public class AngleServo {
    private Servo servo;
    private final String name;
    private final double initAngle;
    private final double range;
    private final LinearOpMode linearOpMode;
    private ElapsedTime timer = new ElapsedTime();
    private double rotateTime;

    /**
     * @param name      name of the servo
     * @param initAngle physical angle correspond to 0 PWM
     * @param range     range of the servo in degree (probably 300 deg)
     * @param opMode    the opMode for hardware map
     */
    public AngleServo(String name, double initAngle, double range, LinearOpMode opMode) {
        this.name = name;
        this.initAngle = initAngle;
        linearOpMode = opMode;
        this.range = range;
    }

    public Servo getServo() {
        return servo;
    }

    public void init() {
        servo = linearOpMode.hardwareMap.get(Servo.class, name);
    }

    public double getAngle() {
        return wrapAngle(range * servo.getPosition() + initAngle, Utility.WRAP_ANGLE_TYPE.zeroTo360);
    }

    public void setAngle(double angle) {
        double to_angle = wrapAngle(angle - initAngle, Utility.WRAP_ANGLE_TYPE.zeroTo360);
        if (to_angle > 300) {
            //Decide if 0 deg (360 deg) or 300 deg is closer
            to_angle = 360 - to_angle < to_angle - 300 ? 0 : 300;
        }
        double targetPWM = (wrapAngle(to_angle, Utility.WRAP_ANGLE_TYPE.zeroTo360)) / range;
        if (Math.abs(targetPWM-servo.getPosition()) > SERVO_ANGLE_PWM_THRESHOLD)
        {
            timer.reset();
            rotateTime = Math.abs(targetPWM - servo.getPosition()) / SERVO_PWM_SPEED;
        }
        servo.setPosition(targetPWM);
    }

    public boolean isRotating ()
    {
        return timer.seconds() >= rotateTime;
    }
}
