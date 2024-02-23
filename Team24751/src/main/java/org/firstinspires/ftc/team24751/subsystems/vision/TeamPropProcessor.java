package org.firstinspires.ftc.team24751.subsystems.vision;

import static org.firstinspires.ftc.team24751.Constants.AllianceColor;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.RIGHT_CLOSE_TEAM_PROP_REGION;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_AREA_THRESHOLD;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_BLUE_MAX;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_BLUE_MIN;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_NAN_COUNT_THRESHOLD;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_RED_MAX;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TEAM_PROP_RED_MIN;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.CENTER;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.LEFT;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.NONE;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropPosition.RIGHT;
import static org.firstinspires.ftc.team24751.Constants.VISION.CV.TeamPropRegion;
import static org.firstinspires.ftc.team24751.Constants.allianceColor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class TeamPropProcessor implements VisionProcessor {
    /**
     * Frame properties
     */
    private int frameWidth = 320;
    private int frameHeight = 240;

    /**
     * Processing variables
     */
    // HSV color threshold. Default is red prop
    private Scalar hsv_min = TEAM_PROP_RED_MIN;
    private Scalar hsv_max = TEAM_PROP_RED_MAX;

    // Kernel for Imgproc.dilate. Created once, use each frame
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(9, 9));

    // Bounding rect
    public Rect boundingRect = new Rect(0, 0, 0, 0);

    // Processed position
    private TeamPropPosition pos = NONE;

    /**
     * Post-process variables
     */
    private Paint paint = new Paint();
    double center;
    double area = 0;
    double rectRatio = 0;
    long nanCount = 0;
    KalmanFilter kalmanFilter = new KalmanFilter(0.4, 0.1, 3);

    /**
     * Init processor
     *
     * @param width       frame width
     * @param height      frame height
     * @param calibration camera calibartion data
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Update frame width and height
        this.frameWidth = width;
        this.frameHeight = height;

        // Update HSV threshold
        if (allianceColor == AllianceColor.BLUE) {
            hsv_min = TEAM_PROP_BLUE_MIN;
            hsv_max = TEAM_PROP_BLUE_MAX;
        } // Else, no need to update, since default is red

        // Init paint
        paint.setColor(Color.RED);
        paint.setTextSize(36);
    }

    /**
     * Process frame
     *
     * @param frame            input frame from camera
     * @param captureTimeNanos exposure time
     * @return
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        try {
            nanCount = Math.min(nanCount, 20);
            // Rotate image
            //Core.rotate(frame.clone(), frame, Core.ROTATE_180);

            // Initial image mat
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

            List<MatOfPoint> passedContours = new ArrayList<>();

            // Filter out contour without correct size ratio
            for (MatOfPoint contour : contours) {
                // Get contour's bounding rect
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                // Filter by bounding box ratio
                this.rectRatio = ((double) rect.size.width / (double) rect.size.height);
                if (this.rectRatio <= 3) {
                    passedContours.add(contour);
                }
            }

            // If no contours is found
            if (passedContours.size() < 1) {
                this.boundingRect = new Rect(0, 0, 0, 0);
                this.center = Double.NaN;
                nanCount++;
                return this.pos = NONE;
            }

            // Find contour with largest area
            double max_area = 0;
            MatOfPoint max_contour = new MatOfPoint();
            for (MatOfPoint contour : passedContours) {
                double area = Imgproc.contourArea(contour);
                if (area >= max_area) {
                    max_area = area;
                    max_contour = contour;
                }
            }

            this.area = max_area;

            // Check if largest area is smaller than threshold
            if (this.area < TEAM_PROP_AREA_THRESHOLD) {
                this.boundingRect = new Rect(0, 0, 0, 0);
                this.center = Double.NaN;
                nanCount++;
                return this.pos = NONE;
            }

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(max_contour.toArray()));
            this.rectRatio = ((double) rect.size.width / (double) rect.size.height);
            this.boundingRect = Imgproc.boundingRect(max_contour);

            // Calculate the center
            center = kalmanFilter.estimate(rect.center.x);
            nanCount--;
            return center;
        } catch (Exception e) {
            e.printStackTrace();
            return center; // I know it's bad but we have to do something instead of crashing
        }
    }

    /**
     * @param canvas               Android canvas to draw on
     * @param onscreenWidth        real onscreen width
     * @param onscreenHeight       real onscreen height
     * @param scaleBmpPxToCanvasPx drawing scale
     * @param scaleCanvasDensity
     * @param userContext
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // If team prop is detected -> draw
        if (this.pos != NONE) {
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
     *
     * @return team prop position. If no team prop is found, return NONE
     */
    public TeamPropPosition getPos() {
        // Get the correct threshold object
        TeamPropRegion regionData;

        // Bot is placed on the right edge of the grid
        regionData = RIGHT_CLOSE_TEAM_PROP_REGION;

        // Decide the position
        if (center < regionData.LEFT_CENTER_POS) return this.pos = LEFT;
        if (center < regionData.CENTER_RIGHT_POS) return this.pos = CENTER;
        if (nanCount >= TEAM_PROP_NAN_COUNT_THRESHOLD) return this.pos = LEFT;
        return this.pos = RIGHT;

    }

    /**
     * Get team prop's center
     *
     * @return team prop's center. If no team prop is found, return 0
     */
    public double getCenter() {
        return center;
    }

    public double getArea() {
        return this.area;
    }

    public double getRectRatio() {
        return this.rectRatio;
    }
}
