package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.DRONE_LAUNCHER;

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
    }
    public void setPosition (double position)
    {
        servo.setPosition(position);
    }
}
