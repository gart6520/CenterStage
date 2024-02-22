package org.firstinspires.ftc.team24751.subsystems.vision.sim;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class PixelAlignProcessor implements VisionProcessor {
    /**
     * Input frame variables
     */
    private int frame_width = 320;
    private int frame_height = 240;
    private CameraCalibration cameraCalibration = null;

    /**
     * Processing variables
     */

    // Field HSV threshold
    private Scalar field_lower_hsv = new Scalar(0, 0, 60);
    private Scalar field_higher_hsv = new Scalar(109, 32, 148);

    // White pixel HSV threshold
    private Scalar white_lower_hsv = new Scalar(20, 5, 166);
    private Scalar white_higher_hsv = new Scalar(102, 43, 255);

    // Purple pixel HSV threshold
    private Scalar purple_lower_hsv = new Scalar(125, 25, 169);
    private Scalar purple_higher_hsv = new Scalar(179, 210, 255);

    // Green pixel HSV threshold
    private Scalar green_lower_hsv = new Scalar(47, 73, 72);
    private Scalar green_higher_hsv = new Scalar(68, 218, 227);

    // Yellow pixel HSV threshold
    private Scalar yellow_lower_hsv = new Scalar(20, 16, 141);
    private Scalar yellow_higher_hsv = new Scalar(36, 255, 255);

    // Minimum object size threshold
    private double min_object_size = 1000;

    // Maximum distance from image bottom threshold
    private double max_distance_from_bottom = 150;

    /**
     * Post-process variables
     */
    private Paint paint = new Paint();
    private List<Rect> res_rect = new ArrayList<>();

    /**
     * @param width frame's width
     * @param height frame's height
     * @param calibration camera calibration information
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Save frame information
        this.frame_width = width;
        this.frame_height = height;
        this.cameraCalibration = calibration;

        // Init paint
        paint.setColor(Color.RED);
        paint.setTextSize(36);
    }

    /**
     * @param frame actual image frame
     * @param captureTimeNanos capture time
     * @return
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Warp it in try/catch. We don't want it to crash in the middle of a competition.
        try {
            // Rotate the image (our camera is up side down)
            Core.rotate(frame.clone(), frame, Core.ROTATE_180);

            // Initial image mat
            Mat img1 = new Mat();

            // Blur the image
            Imgproc.GaussianBlur(frame, img1, new Size(11, 11), 0);

            // Convert image to HSV color space
            Imgproc.cvtColor(img1, img1, Imgproc.COLOR_RGB2HSV);

            // Field's color filter mask
            Mat field_mask = new Mat();

            // Filter only the field in the image
            Core.inRange(img1.clone(), field_lower_hsv, field_lower_hsv, field_mask);

            // Find field's contour
            List<MatOfPoint> field_contours = new ArrayList<>();
            Imgproc.findContours(field_mask, field_contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            // Generate mask for white pixel
            Mat white_mask = new Mat();
            Core.inRange(img1.clone(), white_lower_hsv, white_higher_hsv, white_mask);

            // Generate mask for purple pixel
            Mat purple_mask = new Mat();
            Core.inRange(img1.clone(), purple_lower_hsv, purple_higher_hsv, purple_mask);

            // Generate mask for green pixel
            Mat green_mask = new Mat();
            Core.inRange(img1.clone(), green_lower_hsv, green_higher_hsv, green_mask);

            // Generate mask for yellow pixel
            Mat yellow_mask = new Mat();
            Core.inRange(img1.clone(), yellow_lower_hsv, yellow_higher_hsv, yellow_mask);

            // Combined all the mask together
            Mat pixel_mask = new Mat();
            Core.bitwise_or(white_mask, purple_mask, pixel_mask);
            Core.bitwise_or(pixel_mask, green_mask, pixel_mask);
            Core.bitwise_or(pixel_mask, yellow_mask, pixel_mask);

            // Find contours of all pixel
            List<MatOfPoint> pixel_contours = new ArrayList<>();
            Imgproc.findContours(pixel_mask, pixel_contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // If no object is found -> return now
            if (pixel_contours.size() < 1) {
                res_rect.clear();
                return null;
            }

            // Array holding detected contours and their areas
            TreeMap<Double, MatOfPoint> acm = new TreeMap<>();

            // Integrate over each pixel contours and their sizes
            for (MatOfPoint pixel_contour : pixel_contours) {
                // Calculate pixel contour area
                double pixel_contour_area = Imgproc.contourArea(pixel_contour);

                // Filter contours that does not fit size threshold
                if (pixel_contour_area < min_object_size) {
                    continue;
                }

                // Calculate moments
                Moments M = Imgproc.moments(pixel_contour);

                // Only process non-zero area contour
                if (M.m00 != 0) {
                    // Calculate the center X and Y
                    int cX = (int) Math.round(M.m10 / M.m00);
                    int cY = (int) Math.round(M.m01 / M.m00);

                    // Check if the center of pixel contour is in any field contours
                    for (MatOfPoint field_contour : field_contours) {
                        // Check if:
                        // + pixel contour is in the field contour
                        // + distance from the bottom is met
                        if (Imgproc.pointPolygonTest(new MatOfPoint2f(field_contour.toArray()), new Point(cX, cY), false) >= 0 &&
                            this.frame_height - cY < max_distance_from_bottom) {
                            // Contour meets all the conditions
                            // Add it to detected pixels array
                            acm.put(pixel_contour_area, pixel_contour);
                        }
                    }
                }
            }

            // If after filtering, no object is found -> return
            if (acm.size() < 1) {
                res_rect.clear();
                return null;
            }

            // Get the largest contour object and calculate its bounding box
            Iterator<Map.Entry<Double, MatOfPoint>> ci = acm.descendingMap().entrySet().iterator();
            res_rect.add(Imgproc.boundingRect(ci.next().getValue()));

            // If 2nd object exist
            if (acm.size() > 1) {
                res_rect.add(Imgproc.boundingRect(ci.next().getValue()));
            }

            return null;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
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
        if (this.res_rect.size() > 0) {
            for (Rect r : this.res_rect) {
                // Draw bounding box
                paint.setColor(Color.RED);
                paint.setStyle(Paint.Style.STROKE);
                paint.setStrokeWidth(scaleCanvasDensity * 4);
                canvas.drawRect(makeGraphicsRect(r, scaleBmpPxToCanvasPx), paint);

                // Draw label
                paint.setColor(Color.WHITE);
                paint.setStyle(Paint.Style.FILL);
                paint.setStrokeWidth(scaleCanvasDensity);
                canvas.drawText("pixel", r.x * scaleBmpPxToCanvasPx, r.y * scaleBmpPxToCanvasPx, paint);
            }
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
     * Estimate angle from object
     */
    public List<Double> getObjectAngles() {
        List<Double> angs = new ArrayList<>();

        for (Rect obj : res_rect) {
            // The distance from the camera to the center of the image, in pixels.
            double adjacentSideLength = this.cameraCalibration.focalLengthX;

            // The horizontal distance, from the object to the center of the image, in pixels.
            // This will be negative for objects on the left, positive for objects on the right.
            double oppositeSideLength = (obj.x + (double) obj.width / 2) - 0.5f * this.frame_width;

            double tangent = oppositeSideLength / adjacentSideLength;
            angs.add(Math.toDegrees(Math.atan(tangent)));
        }

        return angs;
    }

    public double getAng() {
        List<Double> angs = this.getObjectAngles();
        double total = 0;
        for (Double ang : angs) {
            total += ang;
        }

        return (total/(double)angs.size());
    }

    /**
     * Get recognition bounding box
     */
    public List<Rect> getRecognition() {
        return this.res_rect;
    }
}
