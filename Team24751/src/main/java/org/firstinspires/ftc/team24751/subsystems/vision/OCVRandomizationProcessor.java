package org.firstinspires.ftc.team24751.subsystems.vision;

import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_BLUE_MAX;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_BLUE_MIN;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_CENTER_RIGHT;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_LEFT_CENTER;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_RED_MAX;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_RED_MIN;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.CENTER;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.LEFT;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.NONE;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.RIGHT;
import static org.firstinspires.ftc.team24751.Constants.allianceColor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.team24751.Constants;
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

public class OCVRandomizationProcessor implements VisionProcessor {
    /**
     * Frame properties
     */
    private int frameWidth = 640;
    private int frameHeight = 480;

    /**
     * Processing variables
     */
    // HSV color threshold. Default is red prop
    private Scalar hsv_min = TEAM_PROP_RED_MIN;
    private Scalar hsv_max = TEAM_PROP_RED_MAX;

    // Bounding rect
    private Rect boundingRect = new Rect(0, 0, 0, 0);

    // Processed position
    private TeamPropPosition pos = NONE;

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
        if (allianceColor == Constants.AllianceColor.BLUE) {
            hsv_min = TEAM_PROP_BLUE_MIN;
            hsv_max = TEAM_PROP_BLUE_MAX;
        } // Else, no need to update, since default is red

        // Init paint
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(8);
        paint.setTextSize(16);
    }

    /**
     * Process frame
     * @param frame input frame from camera
     * @param captureTimeNanos exposure time
     * @return
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Mat holding processing result
        Mat img = new Mat();

        // Blur the image
        Imgproc.GaussianBlur(frame, img, new Size(11, 11), 0);

        // Convert image to HSV color space
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);

        // Filter image by color range
        Mat mask = new Mat();
        Core.inRange(img.clone(), hsv_min, hsv_max, mask);
        Core.bitwise_and(img.clone(), img.clone(), img, mask);

        // Find canny edges
        Imgproc.Canny(img.clone(), img, 30, 200);

        // Close edges
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(9, 9));
        Imgproc.dilate(img.clone(), img, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(img.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // If no contours is found
        if (contours.size() < 1) {
            this.boundingRect = new Rect(0, 0, 0, 0);
            return this.pos = NONE;
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
        if (center < TEAM_PROP_LEFT_CENTER) return this.pos = LEFT;
        if (center < TEAM_PROP_CENTER_RIGHT) return this.pos =  CENTER;
        return this.pos = RIGHT;
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
        if (this.pos != NONE) {
            // Draw bounding box
            canvas.drawRect(makeGraphicsRect(this.boundingRect, scaleBmpPxToCanvasPx), paint);

            // Draw label
            canvas.drawText("pixel", this.boundingRect.x, this.boundingRect.y - 16, paint);
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
