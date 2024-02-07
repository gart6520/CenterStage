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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class OCVRandomizationProcessorTest implements VisionProcessor {
    public enum AllianceColor {
        //Test for code usage in testing
        RED, BLUE, TEST
    }

    public AllianceColor allianceColor = AllianceColor.RED;

    public enum TeamPropPosition {
        NONE, LEFT, CENTER, RIGHT
    }

    /**
     * Frame properties
     */
    private int frameWidth = 640;
    private int frameHeight = 480;

    /**
     * Processing variables
     */
    // HSV color threshold. Default is red prop
    private Scalar hsv_min = new Scalar(0, 115, 133);
    private Scalar hsv_max = new Scalar(5, 228, 255);

    // Kernel for Imgproc.dilate. Created once, use each frame
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(9, 9));

    // Bounding rect
    public Rect boundingRect = new Rect(0, 0, 0, 0);

    // Processed position
    public TeamPropPosition pos = TeamPropPosition.NONE;

    /**
     * Post-process variables
     */
    private Paint paint = new Paint();

    /**
     * Init processor
     * @param width frame width
     * @param height frame height
     * @param calibration camera calibartion data
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Update frame width and height
        this.frameWidth = width;
        this.frameHeight = height;

        // Update HSV threshold
        /*if (allianceColor == AllianceColor.BLUE) {
            hsv_min = TEAM_PROP_BLUE_MIN;
            hsv_max = TEAM_PROP_BLUE_MAX;
        }*/ // Else, no need to update, since default is red

        // Init paint
        paint.setColor(Color.RED);
        //paint.setStyle(Paint.Style.STROKE);
        //paint.setStrokeWidth(8);
        paint.setTextSize(36);
    }

    /**
     * Process frame
     * @param frame input frame from camera
     * @param captureTimeNanos exposure time
     * @return
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Initial image mat, for storing result after blurring
        Mat img1 = new Mat();

        // Blur the image
        Imgproc.GaussianBlur(frame, img1, new Size(11, 11), 0);

        // Convert image to HSV color space
        Imgproc.cvtColor(img1, img1, Imgproc.COLOR_RGB2HSV);

        // Secondary image mat, for storing result after color filter
        Mat img2 = new Mat();

        // Filter image by color range
        Mat mask = new Mat();
        Core.inRange(img1.clone(), hsv_min, hsv_max, mask);
        Core.bitwise_and(img1.clone(), img1.clone(), img2, mask);

        // Find canny edges
        Imgproc.Canny(img2.clone(), img2, 30, 200);

        // Close edges
        Imgproc.dilate(img2, img2, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(img2, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // If no contours is found
        if (contours.size() < 1) {
            this.boundingRect = new Rect(0, 0, 0, 0);
            return this.pos = TeamPropPosition.NONE;
        }

        // Find contour with largest area
        double max_area = 0;
        Mat max_contour = new Mat();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area >= max_area) {
                max_area = area;
                max_contour = contour;
            }
        }

        // Get contour's bounding rect
        this.boundingRect = Imgproc.boundingRect(max_contour);

        // Calculate the center
        int center = this.boundingRect.x + this.boundingRect.width/2;

        // Decide the position
        if (center < 640.0 / 3) return this.pos = TeamPropPosition.LEFT;
        if (center < 2 * 640.0 / 3) return this.pos = TeamPropPosition.CENTER;
        return this.pos = TeamPropPosition.RIGHT;
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
        // If team prop is detected -> draw
        if (this.pos != TeamPropPosition.NONE) {
            // Draw bounding box
            paint.setColor(Color.RED);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(scaleCanvasDensity * 4);
            canvas.drawRect(makeGraphicsRect(this.boundingRect, scaleBmpPxToCanvasPx), paint);

            // Draw label
            paint.setColor(Color.WHITE);
            paint.setStyle(Paint.Style.FILL);
            paint.setStrokeWidth(scaleCanvasDensity);
            canvas.drawText("teamprop", this.boundingRect.x * scaleBmpPxToCanvasPx, this.boundingRect.y * scaleBmpPxToCanvasPx, paint);
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
     * Get team prop position
     * @return team prop position. If no team prop is found, return NONE
     */
    public TeamPropPosition getPos() {
        return this.pos;
    }

    /**
     * Get team prop's center
     * @return team prop's center. If no team prop is found, return 0
     */
    public int getCenter() {
        return this.boundingRect.x + this.boundingRect.width/2;
    }
}
