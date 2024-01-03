package org.firstinspires.ftc.team24751.subsystems.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.external.tfod.CameraInformation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Yolov5Processor implements VisionProcessor {
    // Load native library to use function from C++ file
    static {
        System.loadLibrary("ftcrobotcontroller");
    }

    /*
     * Constants
     */
    private final String model_path = "/sdcard/FIRST/tflitemodels/pixel.tflite"; // tflite weight path
    private final int model_input_size = 320; // Model input size
    private final String[] class_names = {"pixel", "teamprop"}; // Object classes
    private final Map<String, Integer> class_colors = new HashMap<String, Integer>(); // Color for drawing bounding box
    private final float conf_thresh = 0.85f; // Confident threshold. Should be tuned
    private final float iou_thresh = 0.75f; // IOU threshold. Should be tuned

    /*
     * Frame properties
     */
    private int width = 640; // Default width
    private int height = 480; // Default height
    private float x_scale, y_scale; // Scale factor, since our model's input dimension is different from input frame's
    private CameraInformation cameraInformation; // Camera information object. Required by RecognitionImpl
    private final Paint paint = new Paint(); // Android paint object for drawing

    /*
     * Model
     */
    private Interpreter interpreter; // TFLite interpreter
    private Map<Integer, Object> rawOutput;
    private final List<Recognition> recognitions = new ArrayList<>();

    // Raw rnference result class
    private class InferenceRawResult {
        public float[][][][] out1;
        public float[][][][] out2;
        public float[][][][] out3;

        public InferenceRawResult(int inputSize) {
            this.out1 = new float[1][inputSize / 8][inputSize / 8][3 * 85];
            this.out2 = new float[1][inputSize / 16][inputSize / 16][3 * 85];
            this.out3 = new float[1][inputSize / 32][inputSize / 32][3 * 85];
        }
    }

    private final InferenceRawResult rawResult = new InferenceRawResult(model_input_size);

    // This postprocess function comes from C++
    // https://github.com/lp6m/yolov5s_android/blob/master/app/tflite_yolov5_test/app/src/main/cpp/postprocess.cpp
    private native float[][] postprocess(float[][][][] out1, float[][][][] out2, float[][][][] out3, int inputSize, float conf_thresh, float iou_thresh);

    /**
     * @param width       frame width
     * @param height      frame height
     * @param calibration calibration data? (unused)
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Set frame width and height
        this.width = width;
        this.height = height;

        // Calculate xscale, yscale
        this.x_scale = (float) this.width / (float) this.model_input_size;
        this.y_scale = (float) this.height / (float) this.model_input_size;

        // Init cameraInformation object
        this.cameraInformation = new CameraInformation(width, height, 0, calibration.focalLengthX, calibration.focalLengthY);

        // Init TFLite Interpreter
        // Set options for interpreter
        Interpreter.Options options = new Interpreter.Options();
        options.setUseXNNPACK(true); // Use XNNPACK accel lib
        options.setNumThreads(1);    // Number of threads to run. Currently any number > 1 will crash,
        // but that might be just my device's problem/memory problem

        // Load .tflite model
        this.interpreter = new Interpreter(new File(model_path), options);

        // Set paint attributes
        this.paint.setTextSize(16);

        // Set bounding box color for each class
        this.class_colors.put("pixel", Color.RED);
        this.class_colors.put("teamprop", Color.GREEN);
    }

    /**
     * @param frame            input frame
     * @param captureTimeNanos frame capture time? (not used)
     * @return processing result, should be the object's bounding box, class, and score
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Preprocess
        // Create Android Bitmap object
        Bitmap bitmap = Bitmap.createBitmap(this.width, this.height, Bitmap.Config.ARGB_8888);

        // Convert mat to Android's Bitmap format
        Utils.matToBitmap(frame, bitmap);

        // Resize bitmap to fit model's input size
        bitmap = Bitmap.createScaledBitmap(bitmap, model_input_size, model_input_size, true);

        // The implementation of running YoloV5 on TFLite on Java on Android is taken from
        // https://github.com/lp6m/yolov5s_android/blob/master/app/tflite_yolov5_test/app/src/main/java/com/example/tflite_yolov5_test/TfliteRunner.java

        // Create placeholder ByteBuffer for storing the input for TFLite
        // The size for allocating is (input size) * (input size) * no of channel * byte per channel of pixel
        // We are using fp32 model, so byte per channel of pixel should be 4
        // The format is RGB, so no of channel is 3
        ByteBuffer input = ByteBuffer.allocate(model_input_size * model_input_size * 3 * 4);
        input.order(ByteOrder.nativeOrder());
        input.rewind();

        // Normalize the value
        // First, copy the bitmap pixels' value into an int array
        int[] intValues = new int[model_input_size * model_input_size];
        bitmap.getPixels(intValues, 0, model_input_size, 0, 0, model_input_size, model_input_size);

        // Now loop through each pixel and normalize the value in each channel
        for (int i = 0; i < model_input_size; ++i) {
            for (int j = 0; j < model_input_size; ++j) {
                // Get pixel value
                int pixelValue = intValues[i * model_input_size + j];

                // Normalize each channel's value and put the result into 'input' buffer
                input.putFloat((((pixelValue >> 16) & 0xFF)) / 255.0f);
                input.putFloat((((pixelValue >> 8) & 0xFF)) / 255.0f);
                input.putFloat(((pixelValue & 0xFF)) / 255.0f);
            }
        }

        // TensorFlow Lite's time
        // First, reset the output holder
        this.rawOutput = new HashMap<>();
        this.rawOutput.put(0, this.rawResult.out1);
        this.rawOutput.put(1, this.rawResult.out2);
        this.rawOutput.put(2, this.rawResult.out3);

        // Process: now feed the input into the model and run it
        // This should be the function that takes the most time
        this.interpreter.runForMultipleInputsOutputs(new Object[]{input}, rawOutput);

        // Postprocess using C++ code from postprocess.cpp
        // This will filter out incorrect detections
        float[][] bbox_arrs = postprocess(this.rawResult.out1,
                this.rawResult.out2,
                this.rawResult.out3,
                this.model_input_size,
                this.conf_thresh,
                this.iou_thresh);

        // Calculate and add detection result to 'recognitions' array
        for (float[] bbox_arr : bbox_arrs) {
            // Output format note:
            // bbox_arr[0] -> [3]: x1, y1, x2, y2
            // bbox_arr[4]: conf
            // bbox_arr[5]: class id
            this.recognitions.add(
                    new RecognitionImpl(cameraInformation,
                            this.class_names[(int) bbox_arr[5]],
                            bbox_arr[4],
                            new RectF(bbox_arr[0] * this.x_scale,
                                    bbox_arr[1] * this.y_scale,
                                    bbox_arr[2] * this.x_scale,
                                    bbox_arr[3] * this.y_scale)));
        }

        return 1;
    }

    /**
     * Draw bounding box, text label and confidence on to the frame
     *
     * @param canvas               Android canvas to draw to
     * @param onscreenWidth        real Android canvas width on screen
     * @param onscreenHeight       real Android canvas height on screen
     * @param scaleBmpPxToCanvasPx
     * @param scaleCanvasDensity
     * @param userContext
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Draw bounding box for each recognised object
        for (Recognition recognition : recognitions) {
            // Set paint color for object's class
            paint.setColor(this.class_colors.get(recognition.getLabel()));

            // Draw bounding box
            canvas.drawRect(makeGraphicsRect(
                    new Rect(Math.round(recognition.getLeft()),
                            Math.round(recognition.getTop()),
                            Math.round(recognition.getRight()),
                            Math.round(recognition.getBottom())),
                    scaleBmpPxToCanvasPx), paint);

            // Create label text
            String label = recognition.getLabel() + String.format(" %.2f", recognition.getConfidence());

            // Draw label
            canvas.drawText(label, recognition.getLeft(), recognition.getTop() - 16, paint);
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
        int left = Math.round(rect.left * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.top * scaleBmpPxToCanvasPx);
        int right = Math.round(rect.right * scaleBmpPxToCanvasPx);
        int bottom = Math.round(rect.bottom * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * Get recognition result
     *
     * @return TFOD Recognition object
     */
    public List<Recognition> getRecognitions() {
        return this.recognitions;
    }
}
