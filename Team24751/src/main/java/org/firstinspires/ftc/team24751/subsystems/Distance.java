package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.DISTANCE_SENSOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;

public class Distance {
    private DistanceSensor distance;
    private Arm arm;
    private LinearOpMode opMode;

    public Distance(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public DistanceSensor getDistanceSensor() {
        return this.distance;
    }

    public void init() {
//        distance = opMode.hardwareMap.get(DistanceSensor.class, DISTANCE_SENSOR);
    }

    public double getDistanceCM() {
//        return distance.getDistance(DistanceUnit.CM);
        return 10000;
    }

    public double getDistanceMM() {
        return distance.getDistance(DistanceUnit.MM);
    }
}
