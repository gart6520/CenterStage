package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo wrapper that handle get/set angle instead of PWM
 * */
public class AngleServo {
    private Servo servo;
    private final String name;
    private final double initAngle;
    private final double range;
    private final LinearOpMode linearOpMode;

    /**
     * @param name name of the servo
     * @param initAngle physical angle correspond to 0 PWM
     * @param range range of the servo in degree (probably 300 deg)
     * @param opMode the opMode for hardware map
     */
    public AngleServo(String name, double initAngle, double range, LinearOpMode opMode) {
        this.name = name;
        this.initAngle = initAngle;
        linearOpMode = opMode;
        this.range = range;
    }

    public void init() {
        servo = linearOpMode.hardwareMap.get(Servo.class, name);
    }

    public double getAngle() {
        return range * servo.getPosition() + initAngle;
    }

    public void setAngle(double angle) {
        servo.setPosition((angle - initAngle) / range);
    }
}
