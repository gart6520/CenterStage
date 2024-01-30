package org.firstinspires.ftc.team24751.subsystems.sensor;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.POTENTIOMETER;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.POTENTIOMETER.LUT_DATA_FILE_NAME;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.ArrayList;
import java.util.Map;
import java.util.Scanner;
import java.util.TreeMap;

public class Potentiometer {
    AnalogInput potentiometer;
    LinearOpMode opMode;
    Scanner LUTDataReader = new Scanner(LUT_DATA_FILE_NAME);
    //TODO INIT
    TreeMap<Double, Double> LUT = new TreeMap<>();

    public Potentiometer(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        while (LUTDataReader.hasNext())
        {
            double voltage = Double.parseDouble(LUTDataReader.next());
            double angle = Double.parseDouble(LUTDataReader.next());
            LUT.put(voltage, angle);
        }
        potentiometer = opMode.hardwareMap.get(AnalogInput.class, POTENTIOMETER);
    }

    public double getVoltage() {
        return potentiometer.getVoltage();
    }

    double interpolate(double a, double b, double t) {
        return a + t * (b - a);
    }

    public double getAngle() {
        double voltage = getVoltage();
        Map.Entry<Double, Double> ceil = LUT.ceilingEntry(voltage);
        Map.Entry<Double, Double> floor = LUT.floorEntry(voltage);
        if (ceil == null || floor == null) return 0;
        return interpolate(floor.getValue(), ceil.getValue(),
                (voltage - floor.getKey()) / (ceil.getKey() - floor.getKey()));
    }
}
