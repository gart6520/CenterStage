package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.function.Supplier;

public class FuseSensor {
    private double prevX;
    private double prevCovariance;
    double x; // your initial state
    double Q = 0.1; // your model covariance (variance of first sensor)
    double R = 0.4; // your sensor covariance (variance of second sensor)
    double p = 1; // your initial covariance guess
    double K = 1; // your initial Kalman gain guess


    public FuseSensor(double x0) {

        x = x0;
    }

    /**
     * Should call at most once per sensor loop
     */
    public double update(double primarySensor, Double secondarySensor) {
        if (secondarySensor != null) {
            // Secondary sensor available
            x = primarySensor;
            p = prevCovariance + Q;
            K = p / (p + R);
            x = x + K * (secondarySensor - x);
            p = (1 - K) * p;

        } else {
            //Secondary sensor NOT available
            x = primarySensor;
            p = prevCovariance + Q;
//            K = p / (p + R);
//            p = (1 - K) * p;
        }
        prevX = x;
        prevCovariance = p;
        return x;

    }

    public double getX() {
        return x;
    }
}
