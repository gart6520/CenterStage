package org.firstinspires.ftc.team24751.subsystems.vision.sim;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PixelProcessor implements VisionProcessor {
    /**
     * Constants
     */
    // Green pixel
    //public Scalar lower_hsv = new Scalar(0, 69.4, 109.1);
    //public Scalar higher_hsv = new Scalar(72.3, 255, 255);

    // Purple pixel
    public Scalar lower_hsv = new Scalar(153.0, 17.0, 106.3);
    public Scalar higher_hsv = new Scalar(255.0, 255.0, 212.5);

    // White pixel
    //public Scalar lower_hsv = new Scalar(0, 0, 222.4);
    //public Scalar higher_hsv = new Scalar(96.3, 24.1, 255.0);

    /**
     * Frame
     */
    private int frame_width, frame_height;
    private Paint paint = new Paint();

    /**
     * Recognition
     */
    private List<Rect> recognitions = new ArrayList<>();

    /**
     * @param width frame width
     * @param height frame height
     * @param calibration camera calibration data (not used)
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Init frame info
        this.frame_width = width;
        this.frame_height = height;

        // Init paint
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(8);
        paint.setTextSize(16);
    }

    /**
     * @param frame frame to process
     * @param captureTimeNanos frame capture time? (not used)
     * @return OpenCV Rect bounding box
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert image to HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        // Create color filter mask
        Mat mask = new Mat();
        Core.inRange(hsv, this.lower_hsv, this.higher_hsv, mask);

        // Filter the image
        Mat filtered = new Mat();
        Core.bitwise_and(frame, frame, filtered, mask);

        frame = filtered;

        // Find canny edges
        Mat edged = new Mat();
        Imgproc.Canny(filtered, edged, 30, 200);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        // If no object found
        if (contours.size() == 0) {
            recognitions = new ArrayList<>();
            return recognitions;
        }

        // Find object with largest contour area
        MatOfPoint obj = new MatOfPoint();
        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                obj = contour;
            }
        }

        // Get bounding box location
        recognitions = new ArrayList<>();
        recognitions.add(Imgproc.boundingRect(obj));
        return recognitions;
    }

    /**
     * @param canvas Android canvas to draw on
     * @param onscreenWidth real onscreen width
     * @param onscreenHeight real onscreen height
     * @param scaleBmpPxToCanvasPx drawing scale
     * @param scaleCanvasDensity
     * @param userContext
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        for (Rect obj : recognitions) {
            // Draw bounding box
            canvas.drawRect(makeGraphicsRect(obj, scaleBmpPxToCanvasPx), paint);

            // Draw label
            canvas.drawText("pixel", obj.x, obj.y - 16, paint);
        }
    }

    /**
     * Rectangle drawing helper
     *
     * @param rect                 original rectangle
     * @param scaleBmpPxToCanvasPx scale got from onDrawFrame
     * @return Rect object to put to canvas
     */
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * Get pixel recognition
     * @return
     */
    public List<Rect> getRecognitions() {
        return recognitions;
    }
}
