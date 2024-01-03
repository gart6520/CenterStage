package org.firstinspires.ftc.team24751.opmodes.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team24751.subsystems.vision.Yolov5Processor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name = "Test Yolov5 pixel", group = "Test")
public class TestYolov5Pixel extends LinearOpMode {
    private VisionPortal visionPortal;
    private Yolov5Processor yolov5;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        yolov5 = new Yolov5Processor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "fieldCamera"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(yolov5)
                .build();

        waitForStart();
        while(opModeIsActive()) {
            List<Recognition> recognitions = yolov5.getRecognitions();

            for (Recognition recognition : recognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }

            telemetry.update();
        }
    }
}
