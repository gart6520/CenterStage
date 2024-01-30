package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.DRONE_LAUNCHER;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.REV_SERVO_PWM_RANGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class DroneLauncher {
    ServoImplEx servo;
    LinearOpMode opMode;

    public DroneLauncher(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        servo = opMode.hardwareMap.get(ServoImplEx.class, DRONE_LAUNCHER);
        servo.setPwmRange(REV_SERVO_PWM_RANGE);
    }
    public void setPosition (double position)
    {
        servo.setPosition(position);
    }
}
