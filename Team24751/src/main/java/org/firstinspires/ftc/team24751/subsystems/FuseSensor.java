package org.firstinspires.ftc.team24751.subsystems;

public class FuseSensor {
    public static class FuseSensorParameter {
        double Q, R, K;
        /**
         * @param Q covariance of the first sensor
         * @param R covariance of the second sensor
         * @param K kalman gain
         * */
        public FuseSensorParameter(double Q, double R, double K)
        {
            this.Q = Q;
            this.R = R;
            this.K = K;
        }
    }

    private double prevCovariance;
    double x; // your initial state
    double p; // your initial covariance guess
    FuseSensorParameter parameter = new FuseSensorParameter(0.1, 0.4, 1);
    public FuseSensor(double x0, double p0) {

        x = x0;
        p = p0;
    }
    public FuseSensor(double x0, double p0, FuseSensorParameter parameter) {

        x = x0;
        p = p0;
        this.parameter = parameter;
    }

    /**
     * Should call at most once per sensor loop
     */
    public double update(double primarySensor, Double secondarySensor) {
        if (secondarySensor != null) {
            // Secondary sensor available
            x = primarySensor;
            p = prevCovariance + parameter.Q;
            parameter.K = p / (p + parameter.R);
            x = x + parameter.K * (secondarySensor - x);
            p = (1 - parameter.K) * p;

        } else {
            //Secondary sensor NOT available
            x = primarySensor;
            p = prevCovariance + parameter.Q;
//            K = p / (p + R);
//            p = (1 - K) * p;
        }
        prevCovariance = p;
        return x;

    }

    public double getX() {
        return x;
    }
}
