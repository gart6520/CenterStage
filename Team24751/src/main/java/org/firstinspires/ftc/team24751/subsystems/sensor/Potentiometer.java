package org.firstinspires.ftc.team24751.subsystems.sensor;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.POTENTIOMETER;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.POTENTIOMETER.LUT_DATA_FILE_NAME;

import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.sun.source.tree.Tree;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Scanner;
import java.util.TreeMap;

public class Potentiometer {
    AnalogInput potentiometer;
    LinearOpMode opMode;
    Scanner LUTDataReader = new Scanner(new File(LUT_DATA_FILE_NAME));
    //TODO INIT
    TreeMap<Double, Double> LUT = new TreeMap<>();
    KalmanEstimator smoothVoltage = null;

    public Potentiometer(LinearOpMode _opMode) throws FileNotFoundException {
        opMode = _opMode;
    }

    public void init() {
        while (LUTDataReader.hasNextLine()) {
            String line = LUTDataReader.nextLine();
            String[] words = line.split(" ");
            double voltage = Double.parseDouble(words[0]);
            double angle = Double.parseDouble(words[1]);
            LUT.put(voltage, angle);
        }
        potentiometer = opMode.hardwareMap.get(AnalogInput.class, POTENTIOMETER);
        smoothVoltage = new KalmanEstimator(potentiometer::getVoltage, 0.3, 3, 3);
    }

    public double getVoltage() {
        return potentiometer.getVoltage();
    }

    public double getSmoothVoltage() {
        return smoothVoltage.update();
    }

    double interpolate(double a, double b, double t) {
        return a + t * (b - a);
    }

    public double getAngle(boolean smooth) {
        double voltage = getVoltage();
        if (smooth)
            voltage = getSmoothVoltage();
        Map.Entry<Double, Double> ceil = LUT.ceilingEntry(voltage);
        Map.Entry<Double, Double> floor = LUT.floorEntry(voltage);
        if (ceil == null || floor == null) return Double.NEGATIVE_INFINITY;
        if (floor.getKey().doubleValue() == ceil.getKey().doubleValue()) {
            return ceil.getValue();
        }
        return interpolate(floor.getValue(), ceil.getValue(),
                (voltage - floor.getKey()) / (ceil.getKey() - floor.getKey()));
    }

    public void printLUT() {
        LUT.forEach((Double key, Double val) ->
                opMode.telemetry.addLine(key + " " + val));
        opMode.telemetry.update();
    }
}
