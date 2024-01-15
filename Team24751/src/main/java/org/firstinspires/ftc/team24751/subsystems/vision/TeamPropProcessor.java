package org.firstinspires.ftc.team24751.subsystems.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class TeamPropProcessor implements VisionProcessor {
    Scalar minValue = new Scalar(130, 0, 50);
    Scalar maxValue = new Scalar(200, 20, 80);
    Vector2d centerOfDetection = new Vector2d(0, 0);
    ArrayList<MatOfPoint> contours = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat blur = new Mat();
        Mat hierarchy = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, minValue, maxValue, mask);
        Imgproc.blur(mask, blur, new Size(3, 3));
        Imgproc.findContours(blur, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Moments imageMoment = Imgproc.moments(mask, true);
        double x = imageMoment.m10 / imageMoment.m00;
        double y = imageMoment.m01 / imageMoment.m00;
        centerOfDetection = new Vector2d(x, y);
        this.contours = contours;
        return centerOfDetection;
    }

    public Vector2d getCenterOfDetection() {
        return centerOfDetection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Vector2d centDetection = (Vector2d) userContext;
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.FILL);
        for (MatOfPoint matPoint : contours) {
            Point prevPoint = null;
            for (Point point : matPoint.toArray()) {
                if (prevPoint != null) {
                    canvas.drawLine((float) prevPoint.x * scaleBmpPxToCanvasPx, (float) prevPoint.y * scaleBmpPxToCanvasPx,
                            (float) point.x * scaleBmpPxToCanvasPx, (float) point.y * scaleBmpPxToCanvasPx, paint);
                }
                prevPoint = point;
            }
        }
        canvas.drawCircle((float) centDetection.getX() * scaleBmpPxToCanvasPx, (float) centDetection.getY() * scaleBmpPxToCanvasPx, 5, paint);
    }
}
