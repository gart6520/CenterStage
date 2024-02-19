package com.example.meepmeeptesting.Homeostasis;

import java.util.Arrays;

public class LinearRegression {

    public double[] x;
    public double[] y;
    private double m;
    private double b;


    public LinearRegression(double[] y) {
        this.y = y;
        x = new double[y.length];
        for (int i = 0; i < x.length; ++i) {
            x[i] = i;
        }
    }


    public void runLeastSquares() {
        double n = x.length;
        double xySum = 0;
        for (int i = 0; i < x.length; ++i) {
            xySum += x[i] * y[i];
        }
        double m1 = n * xySum - Arrays.stream(x).sum() * Arrays.stream(y).sum();
        double x_squaredSum = 0;
        for (double v : x) {
            x_squaredSum += Math.pow(v, 2);
        }
        double m2 = n * x_squaredSum - Math.pow(Arrays.stream(x).sum(), 2);

        m = m1 / m2;

        b = Arrays.stream(y).sum() - m * Arrays.stream(x).sum();
        b /= n;
    }


    public double predictNextValue() {
        System.out.println("in predict value, length is " + x.length + " m is " + m + " and b is " + b);
        return x.length * m + b;
    }


}
