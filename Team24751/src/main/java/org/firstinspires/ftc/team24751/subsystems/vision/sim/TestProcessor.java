package org.firstinspires.ftc.team24751.subsystems.vision.sim;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestProcessor extends OpenCvPipeline {
    public static volatile Rect detectionRect;

    // Green pixel
    //public Scalar lower_hsv = new Scalar(0, 69.4, 109.1);
    //public Scalar higher_hsv = new Scalar(72.3, 255, 255);

    // Purple pixel
    //public Scalar lower_hsv = new Scalar(153.0, 17.0, 106.3);
    //public Scalar higher_hsv = new Scalar(255.0, 255.0, 212.5);

    // White pixel
    public Scalar lower_hsv = new Scalar(0, 0, 222.4);
    public Scalar higher_hsv = new Scalar(96.3, 24.1, 255.0);

    private Scalar color = new Scalar(255, 0, 0);
    private Mat newMat = new Mat();
    private final List<MatOfPoint> contoursList = new ArrayList<>();

    private Rect JUNCTION = new Rect();

    MatOfPoint m = new MatOfPoint();
    double maxArea = 0, currentArea;
    int i;

    /**
     * Detects the biggest yellow cluster of pixels and puts a green rectangle around it.
     * TODO: something useful
     */

    @Override
    public Mat processFrame(Mat input) {
        // Convert image to HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // Create color filter mask
        Mat mask = new Mat();
        Core.inRange(hsv, this.lower_hsv, this.higher_hsv, mask);

        // Find contours
        Imgproc.findContours(mask, contoursList, newMat, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        JUNCTION.set(null);
        if (contoursList.size() > 0) {
            maxArea = 0;
            for (i = 0; i < contoursList.size(); i++) {
                m = contoursList.get(i);
                currentArea = Imgproc.contourArea(m);
                if (currentArea > maxArea) {
                    maxArea = currentArea;
                    JUNCTION = Imgproc.boundingRect(m);
                }
            }
        }
        detectionRect = JUNCTION;

        if (JUNCTION != null) {
            Imgproc.rectangle(input, JUNCTION, color, 5);
        }

        for (MatOfPoint m : contoursList) {
            m.release();
        }
        contoursList.clear();

        mask.release();

        return input;
    }
}
