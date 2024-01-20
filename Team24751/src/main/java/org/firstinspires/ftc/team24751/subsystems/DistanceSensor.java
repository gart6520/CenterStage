package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.DISTANCE_SENSOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team24751.subsystems.arm.Arm;

public class DistanceSensor {
    private DistanceSensor distance;
    private Arm arm;
    private LinearOpMode opMode;


    public DistanceSensor(DistanceSensor _distance, Arm _arm, LinearOpMode _opMode) {
        distance = _distance;
        arm = _arm;
        opMode = _opMode;
    }
    public DistanceSensor getDistanceSensor() {
        return this.distance;
    }

    public void init() {
        distance = opMode.hardwareMap.get(DistanceSensor.class, DISTANCE_SENSOR);
    }
}
