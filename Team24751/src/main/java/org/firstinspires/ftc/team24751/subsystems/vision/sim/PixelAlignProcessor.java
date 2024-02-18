package org.firstinspires.ftc.team24751.subsystems.vision.sim;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class PixelAlignProcessor implements VisionProcessor {
    /**
     * @param width
     * @param height
     * @param calibration
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    /**
     * @param frame
     * @param captureTimeNanos
     * @return
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    /**
     * @param canvas
     * @param onscreenWidth
     * @param onscreenHeight
     * @param scaleBmpPxToCanvasPx
     * @param scaleCanvasDensity
     * @param userContext
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
